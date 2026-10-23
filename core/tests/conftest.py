import pytest


@pytest.fixture(autouse=True)
def _no_real_speech_key(monkeypatch):
    """core/.env may hold a real SARVAM_API_KEY: tests must never send audio to Sarvam."""
    monkeypatch.setattr("obslam.speech.load_env", lambda name: None)
