#!/usr/bin/env python3
"""Validate submission structure, MJCF vocabulary, metadata, and generated evidence."""

from __future__ import annotations

import json
import py_compile
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REQUIRED = (
    "README.md",
    "JUDGE_BRIEF.md",
    "PR_DESCRIPTION.md",
    "registration.json",
    "requirements.txt",
    "orbital_bayonet_scene.xml",
    "run_orbital_repair.py",
    "rubric_scorecard.json",
    "submission_manifest.json",
)


def fail(message: str, errors: list[str]) -> None:
    errors.append(message)
    print(f"FAIL  {message}")


def pass_check(message: str) -> None:
    print(f"PASS  {message}")


def main() -> int:
    errors: list[str] = []
    for relative in REQUIRED:
        if (ROOT / relative).is_file():
            pass_check(f"present: {relative}")
        else:
            fail(f"missing: {relative}", errors)

    try:
        py_compile.compile(str(ROOT / "run_orbital_repair.py"), doraise=True)
        pass_check("runner compiles")
    except Exception as exc:
        fail(f"runner compile: {exc}", errors)

    try:
        tree = ET.parse(ROOT / "orbital_bayonet_scene.xml")
        root = tree.getroot()
        actuators = root.findall("./actuator/*")
        sensors = root.findall("./sensor/*")
        finger_joints = [joint for joint in root.findall(".//joint") if any(name in joint.get("name", "") for name in ("thumb", "index", "middle", "ring", "little"))]
        checks = {
            "at least 18 actuators": len(actuators) >= 18,
            "at least 13 sensors": len(sensors) >= 13,
            "ten finger joints": len(finger_joints) == 10,
            "external equality constraints": bool(root.findall("./equality/*")),
            "five touch sensors": len(root.findall("./sensor/touch")) >= 5,
        }
        for description, okay in checks.items():
            pass_check(description) if okay else fail(description, errors)
    except Exception as exc:
        fail(f"MJCF XML parse: {exc}", errors)

    try:
        registration = json.loads((ROOT / "registration.json").read_text(encoding="utf-8"))
        uuid = registration.get("uuid", "")
        pr_text = (ROOT / "PR_DESCRIPTION.md").read_text(encoding="utf-8")
        if uuid == "PASTE-YOUR-UUID-HERE":
            fail("registration UUID is still a placeholder", errors)
        elif uuid not in pr_text:
            fail("registration UUID does not match PR description", errors)
        else:
            pass_check("registration UUID matches PR description")
    except Exception as exc:
        fail(f"registration metadata: {exc}", errors)

    artifacts = ROOT / "artifacts"
    if artifacts.exists():
        report_path = artifacts / "report.json"
        video_path = artifacts / "demo.mp4"
        if report_path.is_file():
            report = json.loads(report_path.read_text(encoding="utf-8"))
            pass_check("generated report records success") if report.get("success") else fail("generated report is not successful", errors)
        else:
            fail("artifacts exist but report.json is missing", errors)
        pass_check("generated demo video is non-empty") if video_path.is_file() and video_path.stat().st_size > 10_000 else fail("generated demo video missing or too small", errors)
    else:
        print("INFO  artifacts not generated yet; run the entrypoint")

    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(ROOT / "orbital_bayonet_scene.xml"))
        if model.nu >= 18 and model.nsensor >= 13:
            pass_check(f"MuJoCo loads scene (nq={model.nq}, nu={model.nu}, nsensor={model.nsensor})")
        else:
            fail("loaded MuJoCo inventory is below declared counts", errors)
    except ImportError:
        print("INFO  MuJoCo unavailable; structural XML checks completed")
    except Exception as exc:
        fail(f"MuJoCo scene load: {exc}", errors)

    print(f"\nValidation: {'FAILED' if errors else 'PASSED'} ({len(errors)} error(s))")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
