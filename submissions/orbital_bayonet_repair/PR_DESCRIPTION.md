Registration UUID: d4dc8416-cc0e-4ebb-a255-2372847c3853

## Project summary

- **Project:** Orbital Bayonet Repair
- **Robot:** Procedural 16-channel MuJoCo five-finger service hand
- **Goal:** Restore a keyed satellite power connector, lock it, proof-test it, verify power, and export labeled evidence
- **Control:** Deterministic task plan plus closed-loop visual/contact residual controller
- **Highlights:** 18 actuators, 13 sensors, five-finger coordination, injected calibration fault, 8+ N proof load, 32-rollout ablation, generated one-minute video
- **Limitations:** Authored stage plan; plug begins constrained in-hand; ablation isolates controller dynamics

## Run

```bash
python3 -m pip install -r requirements.txt
python submissions/orbital_bayonet_repair/run_orbital_repair.py
python submissions/orbital_bayonet_repair/validate_submission.py
```

