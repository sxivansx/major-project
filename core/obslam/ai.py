"""AI layer over the findings: Jev (TypeSafe, typed decisions) for every judgment call, Gemini Flash
for prose. Jev goes to TypeSafe directly (TYPESAFE_API_KEY) or to OpenRouter; Gemini goes through
OpenRouter (OPENROUTER_API_KEY). Inputs are the findings dict, never points.

Guardrails: Jev cannot write text, so every ranking reason is a template over the finding's own
numbers. Prose is checked after generation; if it claims a person ("survivor"), the deterministic
template is returned instead. Without a key everything degrades to templates and `available: false`.
"""

from __future__ import annotations

import json
import logging
import os
import re
from pathlib import Path
from typing import Callable

import httpx

log = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parents[2]
ENV_FILE = ROOT / "core" / ".env"
OPENROUTER = "https://openrouter.ai/api"
TYPESAFE = "https://api.typesafe.ai"
JEV_USD_PER_INPUT_TOKEN = 0.042 / 1e6  # TypeSafe list price; its API reports tokens, not cost
PROSE_MODEL = os.environ.get("OBSLAM_PROSE_MODEL", "google/gemini-3.5-flash")

FORBIDDEN = re.compile(r"\b(survivor|victim|casualt|body|bodies|person is|people are)\b", re.I)


class AIError(Exception):
    pass


def load_env(name: str) -> str | None:
    val = os.environ.get(name)
    if val:
        return val.strip()
    if ENV_FILE.exists():
        for line in ENV_FILE.read_text().splitlines():
            line = line.strip()
            if line.startswith(f"{name}="):
                return line.split("=", 1)[1].strip().strip('"').strip("'") or None
    return None


class AIClient:
    """Decisions go to TypeSafe directly when TYPESAFE_API_KEY is set, else to Jev on OpenRouter.
    Prose always goes to OpenRouter. Either key may be missing; each half degrades on its own."""

    def __init__(
        self,
        key: str | None = None,
        typesafe_key: str | None = None,
        decisions_model: str | None = None,
        prose_model: str = PROSE_MODEL,
        transport: httpx.BaseTransport | None = None,
        timeout: float = 40.0,
    ):
        self.key = key if key is not None else load_env("OPENROUTER_API_KEY")
        self.typesafe_key = typesafe_key if typesafe_key is not None else load_env("TYPESAFE_API_KEY")
        default_model = "jev-latest" if self.typesafe_key else "typesafe/jev-1.14"
        self.decisions_model = decisions_model or os.environ.get("OBSLAM_DECISIONS_MODEL") or default_model
        self.prose_model = prose_model
        self._http = httpx.Client(timeout=timeout, transport=transport)

    @property
    def decisions_available(self) -> bool:
        return bool(self.typesafe_key or self.key)

    @property
    def prose_available(self) -> bool:
        return bool(self.key)

    @property
    def available(self) -> bool:
        return self.decisions_available

    def status(self) -> dict:
        provider = "typesafe" if self.typesafe_key else ("openrouter" if self.key else None)
        reasons = []
        if not self.decisions_available:
            reasons.append("no TYPESAFE_API_KEY or OPENROUTER_API_KEY for Jev")
        if not self.prose_available:
            reasons.append("no OPENROUTER_API_KEY for Gemini (sitrep and answers use templates)")
        return {
            "available": self.decisions_available,
            "reason": "; ".join(reasons) if reasons else None,
            "models": {"decisions": self.decisions_model, "prose": self.prose_model},
            "decisions": {"available": self.decisions_available, "provider": provider, "model": self.decisions_model},
            "prose": {"available": self.prose_available, "provider": "openrouter" if self.key else None, "model": self.prose_model},
        }

    def _post(self, url: str, body: dict, token: str, who: str) -> dict:
        headers = {"Authorization": f"Bearer {token}", "HTTP-Referer": "https://github.com/sxivansx/major-project", "X-Title": "major-project"}
        try:
            r = self._http.post(url, json=body, headers=headers)
        except httpx.HTTPError as e:
            raise AIError(f"{who} unreachable: {e}") from e
        if r.status_code >= 400:
            raise AIError(f"{who} {r.status_code}: {r.text[:300]}")
        return r.json()

    def decide(self, state, questions: dict) -> tuple[dict, float]:
        """Jev: one state, many typed questions. Returns (answers by name, cost in USD)."""
        if not self.decisions_available:
            raise AIError("AI layer unavailable: no TYPESAFE_API_KEY or OPENROUTER_API_KEY")
        body = {"model": self.decisions_model, "state": state, "questions": questions}
        if self.typesafe_key:
            data = self._post(f"{TYPESAFE}/v1/systemone", body, self.typesafe_key, "TypeSafe")
            usage = data.get("usage") or {}
            cost = float(usage.get("cost") or (usage.get("input_tokens") or 0) * JEV_USD_PER_INPUT_TOKEN)
        else:
            data = self._post(f"{OPENROUTER}/alpha/decisions", body, self.key, "OpenRouter")
            cost = float((data.get("usage") or {}).get("cost") or 0.0)
        return data.get("answers", {}), cost

    def complete(self, system: str, user: str, max_tokens: int = 1200, temperature: float = 0.2) -> tuple[str, float]:
        """Gemini 3.5 Flash reasons before answering and the reasoning counts against max_tokens, so
        ask for minimal effort (it cannot be disabled), drop the reasoning from the reply, and treat a
        truncated reply as a failure. Measured 22 Oct 2026: minimal → 0 reasoning tokens, ~$0.001/call."""
        if not self.prose_available:
            raise AIError("prose unavailable: no OPENROUTER_API_KEY")
        data = self._post(
            f"{OPENROUTER}/v1/chat/completions",
            {
                "model": self.prose_model,
                "messages": [{"role": "system", "content": system}, {"role": "user", "content": user}],
                "max_tokens": max_tokens,
                "temperature": temperature,
                "reasoning": {"effort": "minimal", "exclude": True},  # 0 reasoning tokens on gemini-3.5-flash; "low" burns ~400+
                "usage": {"include": True},
            },
            self.key,
            "OpenRouter",
        )
        try:
            choice = data["choices"][0]
            text = choice["message"]["content"]
        except (KeyError, IndexError, TypeError) as e:
            raise AIError(f"unexpected completion response: {str(data)[:200]}") from e
        cost = float((data.get("usage") or {}).get("cost") or 0.0)
        if choice.get("finish_reason") == "length" or not (text or "").strip():
            raise AIError(f"completion truncated or empty (finish_reason={choice.get('finish_reason')})")
        return text.strip(), cost


# ---- facts --------------------------------------------------------------------------------------
CONTEXT = (
    "Context: a stationary tilt-LiDAR scanner on the ground in a collapsed-structure search area produced a "
    "2.5D height map. Bearings are degrees clockwise from the scanner's forward mark; distances in metres from "
    "the scanner. A 'void' is a slab or debris with open space beneath it that a person could fit in; it says "
    "nothing about whether anyone is there. 'Shadow' regions are hidden behind nearer objects; the 'blind stripe' "
    "is a band along the scanner's tilt axle that this design cannot see. The scanner cannot see under things."
)


def facts_from(findings: dict) -> dict:
    """The subset the models see: no grid arrays, capped lists."""
    keep = ("scanner", "points", "floor", "coverage", "traversability", "next_station")
    f = {k: findings.get(k) for k in keep}
    for k in ("voids", "obstacles", "depressions", "shadows"):
        f[k] = [{kk: vv for kk, vv in item.items() if kk != "bbox_m"} for item in findings.get(k, [])[:12]]
    return f


def describe(kind: str, item: dict) -> str:
    if kind == "obstacle" and item.get("wall"):
        kind = "wall"
    val = {
        "void": f"covered void, clearance {item.get('clearance_m')} m",
        "obstacle": f"obstacle {item.get('height_m')} m high",
        "wall": f"wall-like structure {item.get('height_m')} m high (boundary, not debris)",
        "depression": f"depression {item.get('depth_m')} m deep",
        "shadow": "unseen region",
    }[kind]
    return f"{item['id']}: {val}, {item['area_m2']} m², {item['distance_m']} m away at {item['bearing_deg']:.0f}°"


# ---- triage ----------------------------------------------------------------------------------------
LEVELS = ["Routine terrain or noise; no action.", "Worth a look if time allows.", "Investigate first; it could hide a person or block access."]


def triage(ai: AIClient, findings: dict, max_items: int = 24) -> dict:
    facts = facts_from(findings)
    items: list[tuple[str, dict]] = []
    for kind, key in (("void", "voids"), ("depression", "depressions"), ("obstacle", "obstacles"), ("shadow", "shadows")):
        for it in facts[key]:
            items.append((kind, it))
    items = items[:max_items]
    questions = {
        "rescan": {
            "type": "noul",
            "instructions": "Should the operator rescan from another position before acting on this map?",
            "criteria": {
                "true": "Coverage is poor, large shadow or unknown areas or the blind stripe could hide what matters, or the floor fit is unreliable.",
                "false": "The map covers the area well enough to act on now.",
            },
        }
    }
    for kind, it in items:
        questions[f"att_{it['id']}"] = {
            "type": "score",
            "instructions": f"How much attention should a rescuer give {describe(kind, it)}?",
            "criteria": LEVELS,
        }
    answers, cost = ai.decide({"context": CONTEXT, "findings": facts}, questions)
    ranked = []
    for kind, it in items:
        a = answers.get(f"att_{it['id']}", {})
        score = float(a.get("score", 0.0))
        ranked.append(
            {
                "id": it["id"],
                "kind": "wall" if kind == "obstacle" and it.get("wall") else kind,
                "attention": round(min(1.0, max(0.0, score / (len(LEVELS) - 1))), 3),
                "confidence": round(float(a.get("confidence", 0.0)), 3),
                "why": describe(kind, it),
            }
        )
    ranked.sort(key=lambda r: -r["attention"])
    p = float(answers.get("rescan", {}).get("noul", 0.0))
    return {
        "items": ranked,
        "rescan": {"recommended": p >= 0.5, "p": round(p, 3), "confidence": round(abs(p - 0.5) * 2, 3)},
        "model": ai.decisions_model,
        "cost_usd": round(cost, 6),
    }


# ---- sitrep --------------------------------------------------------------------------------------
SITREP_SYSTEM = (
    "You write a five-line situation report for a search-and-rescue operator from measured facts. Rules: "
    "use only numbers that appear in the facts; never claim a person is present, say 'possible void' or "
    "'unseen region'; never use the words survivor, victim, body; give bearings and distances as in the facts; "
    "state what the scan could not see (shadow, blind stripe, unknown) and the suggested next scanner position "
    "if any; plain sentences, no headers, no bullet symbols, no markdown, at most five lines."
)


def template_sitrep(findings: dict, triage_result: dict | None = None) -> str:
    f = facts_from(findings)
    cov, fl = f["coverage"] or {}, f["floor"] or {}
    lines = [
        f"Scan of {cov.get('area_seen_m2', 0)} m² at {cov.get('coverage_pct', 0)}% coverage; floor fit RMS {fl.get('rms_mm', '?')} mm, tilt {fl.get('tilt_deg', '?')}°."
    ]
    if f["voids"]:
        v = f["voids"][0]
        lines.append(f"{len(f['voids'])} possible void(s); largest {v['id']} {v['area_m2']} m² with {v['clearance_m']} m clearance, {v['distance_m']} m at {v['bearing_deg']:.0f}°.")
    else:
        lines.append("No covered voids detected in the scanned area.")
    if f["obstacles"]:
        o = f["obstacles"][0]
        lines.append(f"{len(f['obstacles'])} obstacle(s); largest {o['id']} {o['height_m']} m high, {o['distance_m']} m at {o['bearing_deg']:.0f}°.")
    unseen = f"{cov.get('shadow_cells', 0)} shadow, {cov.get('blind_cells', 0)} blind and {cov.get('unknown_cells', 0)} unknown cells not seen"
    ns = f.get("next_station")
    if ns and ns.get("bearing_deg") is not None:
        lines.append(f"{unseen}; next scanner position suggested {ns['distance_m']} m at {ns['bearing_deg']:.0f}°.")
    elif ns:
        lines.append(f"{unseen}; {ns['reason']}.")
    else:
        lines.append(f"{unseen}.")
    if triage_result and triage_result.get("rescan", {}).get("recommended"):
        lines.append(f"Assistant verdict: rescan before acting (p={triage_result['rescan']['p']}).")
    return "\n".join(lines[:5])


def sitrep(ai: AIClient, findings: dict, triage_result: dict | None = None) -> dict:
    facts = facts_from(findings)
    fallback = template_sitrep(findings, triage_result)
    if not ai.prose_available:
        return {"text": fallback, "facts": facts, "model": "template", "cost_usd": 0.0, "note": ai.status()["reason"]}
    user = json.dumps({"context": CONTEXT, "facts": facts, "assistant_ranking": (triage_result or {}).get("items", [])[:6], "rescan": (triage_result or {}).get("rescan")})
    try:
        text, cost = ai.complete(SITREP_SYSTEM, user)
    except AIError as e:
        return {"text": fallback, "facts": facts, "model": "template", "cost_usd": 0.0, "note": str(e)}
    if not text or FORBIDDEN.search(text) or len(text.splitlines()) > 6:
        log.warning("sitrep rejected by the guardrail, returning the template")
        return {"text": fallback, "facts": facts, "model": "template", "cost_usd": round(cost, 6), "note": "model output rejected by the guardrail"}
    return {"text": text, "facts": facts, "model": ai.prose_model, "cost_usd": round(cost, 6)}


# ---- ask the map -----------------------------------------------------------------------------------
INTENTS = {
    "route": "Asks whether or how to reach a place or finding, a path, a way through, access.",
    "measure": "Asks how far, which direction, how big or how high a specific finding is.",
    "list_voids": "Asks about voids, cavities, gaps, places where someone could be.",
    "list_obstacles": "Asks about obstacles, debris, walls, what is in the way.",
    "coverage": "Asks what was not seen, how complete the scan is, where to scan next.",
    "sitrep": "Asks for a summary, overview, situation report, what we have.",
    "other": "Anything else.",
}

ANSWER_SYSTEM = (
    "Answer a search-and-rescue operator's question in at most three plain sentences using only the tool "
    "result and facts given, and include the relevant numbers (distance, bearing, length, size, clearance). "
    "Never claim a person is present; never use the words survivor, victim, body. "
    "If the tool result says something was not found or not possible, say so plainly. No markdown."
)


def template_answer(intent: str, tool: dict) -> str:
    """Plain-words answer from the tool result when no prose model is configured."""
    if intent == "route":
        r = tool.get("route") or {}
        if r.get("found"):
            return f"Route to {tool.get('target')}: {r['length_m']} m, {r['verdict']}, max step {r['max_step_m']} m."
        return f"No passable route to {tool.get('target')}: {r.get('reason', 'unknown')}."
    if intent == "measure":
        m = tool.get("measure") or {}
        return f"{m.get('id')}: {m.get('distance_m')} m away at {m.get('bearing_deg')}°, {m.get('area_m2')} m²."
    if intent == "list_voids":
        v = tool.get("voids") or []
        return f"{len(v)} possible void(s): " + "; ".join(f"{i['id']} {i['area_m2']} m² at {i['distance_m']} m / {i['bearing_deg']:.0f}°" for i in v[:5]) if v else "No covered voids detected."
    if intent == "list_obstacles":
        o = tool.get("obstacles") or []
        return f"{len(o)} obstacle(s): " + "; ".join(f"{i['id']} {i['height_m']} m high at {i['distance_m']} m / {i['bearing_deg']:.0f}°" for i in o[:5]) if o else "No obstacles detected."
    if intent == "coverage":
        c = tool.get("coverage") or {}
        ns = tool.get("next_station")
        return f"Coverage {c.get('coverage_pct')}% of the observable area; {c.get('shadow_cells')} shadow, {c.get('blind_cells')} blind, {c.get('unknown_cells')} unknown cells." + (f" Next station: {ns['reason']}." if ns else "")
    if intent == "sitrep":
        return tool.get("sitrep", "")
    return "No prose model configured; see the tool result."


def ask(ai: AIClient, question: str, findings: dict, route_to: Callable[[float, float], dict | None]) -> dict:
    facts = facts_from(findings)
    ids = [it["id"] for k in ("voids", "obstacles", "depressions", "shadows") for it in facts[k]]
    by_id = {it["id"]: (k, it) for k in ("voids", "obstacles", "depressions", "shadows") for it in facts[k]}
    questions = {"intent": {"type": "choice", "instructions": "What does the operator's question ask for?", "criteria": INTENTS}}
    if ids:
        questions["target"] = {
            "type": "choice",
            "instructions": "Which finding, if any, does the question refer to?",
            "criteria": {**{i: describe(by_id[i][0].rstrip("s") if by_id[i][0] != "voids" else "void", by_id[i][1]) for i in ids}, "none": "No specific finding."},
        }
    answers, cost = ai.decide({"context": CONTEXT, "question": question, "findings": facts}, questions)
    intent = answers.get("intent", {}).get("choice", "other")
    conf = float(answers.get("intent", {}).get("confidence", 0.0))
    target = answers.get("target", {}).get("choice", "none") if ids else "none"
    tool: dict = {"intent": intent, "target": target if target != "none" else None}
    if intent == "route" and target in by_id:
        it = by_id[target][1]
        tool["route"] = route_to(it["x_m"], it["y_m"]) or {"found": False, "reason": "no passable route"}
    elif intent == "measure" and target in by_id:
        it = by_id[target][1]
        tool["measure"] = {k: it.get(k) for k in ("id", "distance_m", "bearing_deg", "area_m2", "clearance_m", "height_m", "depth_m") if it.get(k) is not None}
    elif intent == "list_voids":
        tool["voids"] = facts["voids"]
    elif intent == "list_obstacles":
        tool["obstacles"] = facts["obstacles"]
    elif intent == "coverage":
        tool["coverage"], tool["next_station"] = facts["coverage"], facts["next_station"]
    elif intent == "sitrep":
        tool["sitrep"] = template_sitrep(findings)
    else:
        tool["facts"] = facts
    if ai.prose_available:
        try:
            answer, cost2 = ai.complete(ANSWER_SYSTEM, json.dumps({"question": question, "tool_result": tool}), max_tokens=900)
            cost += cost2
            if FORBIDDEN.search(answer):
                answer = "Assistant answer withheld by the guardrail; see the tool result."
        except AIError as e:
            log.warning("answer prose failed, using the template: %s", e)
            answer = template_answer(intent, tool)
    else:
        answer = template_answer(intent, tool)
    return {"intent": intent, "confidence": round(conf, 3), "target": tool["target"], "tool": tool, "answer": answer, "cost_usd": round(cost, 6)}
