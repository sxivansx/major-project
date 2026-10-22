"""AI layer against a fake OpenRouter (httpx.MockTransport): no network, deterministic answers."""

import json
import time

import httpx
import pytest
from fastapi.testclient import TestClient

from obslam import ai as ai_mod
from obslam.ai import AIClient, AIError, FORBIDDEN, template_sitrep
from obslam.server import create_app


def fake_openrouter(prose: str = "Scan covers 12.5 m² at 83% coverage. One possible void V1 at 1.4 m, bearing 45°."):
    calls = []

    def handler(request: httpx.Request) -> httpx.Response:
        body = json.loads(request.content)
        calls.append((request.url.path, body))
        if request.url.path.endswith("/alpha/decisions"):
            answers = {}
            state = json.dumps(body["state"]).lower()
            question = str(body["state"].get("question", "")).lower() if isinstance(body["state"], dict) else ""
            for name, q in body["questions"].items():
                if q["type"] == "noul":
                    answers[name] = {"type": "noul", "noul": 0.7 if "coverage_pct\": 5" in state else 0.2}
                elif q["type"] == "score":
                    answers[name] = {"type": "score", "score": 1.6 if name.startswith("att_V") else 0.4, "confidence": 0.8, "legend": {}, "probabilities": {}}
                elif q["type"] == "choice":
                    opts = list(q["criteria"])
                    pick = next((o for o in opts if o.lower() in question and o != "none"), opts[0])
                    if name == "intent":
                        pick = "route" if "reach" in question or "route" in question else ("sitrep" if "summary" in question else "other")
                    answers[name] = {"type": "choice", "choice": pick, "confidence": 0.9, "probabilities": {o: (0.9 if o == pick else 0.1) for o in opts}}
            return httpx.Response(200, json={"id": "x", "model": body["model"], "answers": answers, "usage": {"input_tokens": 500, "output_tokens": 10, "cost": 0.00002}})
        if request.url.path.endswith("/v1/chat/completions"):
            return httpx.Response(200, json={"choices": [{"message": {"content": prose}}], "usage": {"cost": 0.0001}})
        return httpx.Response(404, json={"error": "no"})

    return httpx.MockTransport(handler), calls


def client_with(transport):
    return AIClient(key="test-key", transport=transport)


@pytest.fixture
def app_client(tmp_path):
    transport, calls = fake_openrouter()
    app = create_app(config_path=tmp_path / "c.json", sessions_dir=tmp_path / "s", web_dist=tmp_path / "nope", sim_hz=60, ai_client=client_with(transport))
    with TestClient(app) as c:
        c.calls = calls
        yield c


def run_sim_sweep(c):
    c.post("/api/servo/connect", json={"port": "sim"})
    c.post("/api/lidar/connect", json={"source": "sim"})
    c.post("/api/job/start", json={"name": "ai", "phi_start": -40, "phi_end": 40, "phi_step": 5, "settle_ms": 0})
    deadline = time.time() + 10
    while time.time() < deadline and c.get("/api/status").json()["job"]["state"] == "running":
        time.sleep(0.05)


def test_unavailable_without_key(tmp_path):
    ai = AIClient(key="", transport=httpx.MockTransport(lambda r: httpx.Response(500)))
    assert ai.available is False and ai.status()["available"] is False
    with pytest.raises(AIError, match="unavailable"):
        ai.decide("x", {})
    app = create_app(config_path=tmp_path / "c.json", sessions_dir=tmp_path / "s", web_dist=tmp_path / "nope", ai_client=ai)
    with TestClient(app) as c:
        assert c.get("/api/ai/status").json()["available"] is False
        assert c.post("/api/ai/triage", json={}).status_code == 503
        r = c.post("/api/ai/sitrep", json={}).json()
        assert r["model"] == "template" and "coverage" in r["text"]


def test_triage_ranks_voids_first_and_reads_rescan(app_client):
    run_sim_sweep(app_client)
    r = app_client.post("/api/ai/triage", json={"cell": 0.1})
    assert r.status_code == 200, r.text
    t = r.json()
    assert t["items"], "the sim room has obstacles and shadows to rank"
    assert all(0 <= it["attention"] <= 1 and 0 <= it["confidence"] <= 1 for it in t["items"])
    assert t["items"] == sorted(t["items"], key=lambda i: -i["attention"])
    assert t["rescan"]["recommended"] in (True, False) and t["cost_usd"] > 0
    path, body = app_client.calls[-1]
    assert path.endswith("/alpha/decisions") and body["model"].startswith("typesafe/jev")
    assert "rescan" in body["questions"] and all(v["type"] in ("noul", "score") for v in body["questions"].values())
    assert "points" in body["state"]["findings"] and "bbox_m" not in json.dumps(body["state"])


def test_sitrep_uses_model_and_guardrail(app_client):
    run_sim_sweep(app_client)
    r = app_client.post("/api/ai/sitrep", json={}).json()
    assert r["model"].startswith("google/") and "possible void" in r["text"] and r["cost_usd"] > 0
    assert "facts" in r and "coverage" in r["facts"]


def test_sitrep_rejects_forbidden_words(tmp_path):
    transport, _ = fake_openrouter(prose="A survivor is trapped at 1.4 m bearing 45°.")
    ai = client_with(transport)
    findings = {"scanner": {}, "points": 10, "floor": {"rms_mm": 9, "tilt_deg": 0.1}, "coverage": {"area_seen_m2": 3, "coverage_pct": 50, "shadow_cells": 1, "blind_cells": 0, "unknown_cells": 2}, "traversability": {}, "voids": [], "obstacles": [], "depressions": [], "shadows": [], "next_station": None}
    r = ai_mod.sitrep(ai, findings)
    assert r["model"] == "template" and not FORBIDDEN.search(r["text"]) and "guardrail" in r["note"]


def test_template_sitrep_is_five_lines_max():
    findings = {"scanner": {}, "points": 0, "floor": {}, "coverage": {}, "traversability": {}, "voids": [], "obstacles": [], "depressions": [], "shadows": [], "next_station": None}
    t = template_sitrep(findings)
    assert 1 <= len(t.splitlines()) <= 5 and "No covered voids" in t


def test_ask_routes_to_a_tool(app_client):
    run_sim_sweep(app_client)
    findings = app_client.get("/api/ops/findings?cell=0.1").json()
    target = findings["obstacles"][0]["id"]
    r = app_client.post("/api/ai/ask", json={"question": f"can we reach {target}?", "cell": 0.1})
    assert r.status_code == 200, r.text
    a = r.json()
    assert a["intent"] == "route" and a["target"] == target and "route" in a["tool"]
    assert "found" in a["tool"]["route"] and a["answer"]
    r2 = app_client.post("/api/ai/ask", json={"question": "give me a summary"}).json()
    assert r2["intent"] == "sitrep" and "sitrep" in r2["tool"]
    assert app_client.post("/api/ai/ask", json={"question": "  "}).status_code == 400


def test_openrouter_errors_become_502(tmp_path):
    ai = AIClient(key="k", transport=httpx.MockTransport(lambda r: httpx.Response(429, json={"error": "rate"})))
    app = create_app(config_path=tmp_path / "c.json", sessions_dir=tmp_path / "s", web_dist=tmp_path / "nope", ai_client=ai)
    with TestClient(app) as c:
        r = c.post("/api/ai/triage", json={})
        assert r.status_code == 502 and "429" in r.json()["error"]
        s = c.post("/api/ai/sitrep", json={}).json()  # falls back to the template, never fails
        assert s["model"] == "template"
