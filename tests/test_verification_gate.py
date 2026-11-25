"""The user's required verification order must also hold for programmatic calls."""
import json
import pytest
from flask_repro import verification_gate


def test_showcase_is_locked_without_complete_evidence(tmp_path,monkeypatch):
    path=tmp_path/'gate.json';monkeypatch.setattr(verification_gate,'GATE',path)
    with pytest.raises(RuntimeError,match='locked'):verification_gate.require_verified_reproduction('Rendering')
    path.write_text(json.dumps(dict(criteria={name:dict(passed=False) for name in verification_gate.REQUIRED},verified_source_sha256={})))
    with pytest.raises(RuntimeError,match='native_algorithm_contracts'):verification_gate.require_verified_reproduction('Simulation')


def test_changed_source_invalidates_previously_verified_evidence(tmp_path,monkeypatch):
    path=tmp_path/'gate.json';source=tmp_path/'core.cc';source.write_text('changed source')
    monkeypatch.setattr(verification_gate,'GATE',path);monkeypatch.setattr(verification_gate,'ROOT',tmp_path)
    path.write_text(json.dumps(dict(criteria={name:dict(passed=True) for name in verification_gate.REQUIRED},verified_source_sha256={'core.cc':'incorrect_digest'})))
    with pytest.raises(RuntimeError,match='source_changed:core.cc'):verification_gate.require_verified_reproduction('Rendering')
