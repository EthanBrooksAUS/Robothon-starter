# 🤖 Orbital Bayonet Repair

**FFAI Robothon 2026** — Freestyle Category

> **A 16-channel, five-finger MuJoCo service hand repairs a damaged satellite power connector through closed-loop visual servoing, five-contact grasp balancing, compliant keyed insertion, bayonet locking, proof-load recovery, and guarded power verification—achieving all 9 task stages and 100% success across 32 fixed-seed residual-controller rollouts.**

---

## 📋 Project Overview

This project implements a self-contained dexterous-manipulation task for satellite power-system servicing in microgravity. The system combines:

- **Closed-Loop Visual Servoing**: Frame-sensor feedback rejects an injected camera-calibration bias during alignment
- **Five-Finger Contact Balancing**: Independent fingertip measurements adapt grasp closure and preserve contact stability
- **Compliant Keyed Insertion**: Residual position corrections reduce connector alignment error before locking
- **Bayonet Lock and Proof Test**: The collar rotates to its mechanical stop before an external load validates the repair
- **Guarded Power Verification**: The verification channel is pressed only after insertion, locking, and proof testing

### Key Achievements

- **9/9 task stages completed** (100% task completion)
- **6/6 terminal success conditions passed**
- **Minimum insertion error**: 4.042 mm
- **Median servo-error reduction**: 82.0%
- **Proof load sustained**: 8.9987 N
- **Closed-loop evaluation**: 32/32 successful rollouts

---

## 🎯 Task Summary (9/9 Completed)

| # | Task | Type | Description |
|---|---|---|---|
| 1 | Sensor Self-Check | Validation | Boot the sensors and inspect the damaged power panel |
| 2 | Connector Approach | Positioning | Approach with all five fingers pre-shaped around the plug |
| 3 | Five-Finger Grasp | Dexterity | Close ten finger joints and balance measured fingertip contact |
| 4 | Visual-Servo Recovery | Closed-Loop Control | Reject the injected camera-calibration bias using frame feedback |
| 5 | Keyed Insertion | Precision Assembly | Align the connector key and perform compliant insertion |
| 6 | Bayonet Lock | Mechanical Assembly | Rotate the locking collar beyond 1.20 rad |
| 7 | Proof-Load Test | Disturbance Recovery | Apply an external load above 8 N and recover displacement |
| 8 | Power Verification | Conditional Control | Depress the guarded verification channel beyond 25 mm |
| 9 | Dataset Export | Evidence Generation | Hold a safe pose and export states, actions, contacts, and labels |

---

## 🔬 Technical Innovations

### 1. Stage Prior with Closed-Loop Residual Correction

```python
tracking_error = ideal_tip - measured_plug_position
residual = clip(0.82 * tracking_error, residual_limits)
corrected_target = nominal_target + injected_bias + residual
```

- The deterministic stage plan provides reproducible nominal motion
- The residual layer reads actual MuJoCo frame sensors on every control step
- Corrections are bounded to retain stable, physically interpretable behavior

### 2. Reproducible Calibration-Fault Rejection

- A deterministic lateral and vertical camera bias is injected during approach and alignment
- Raw and corrected servo errors are recorded independently
- Median error falls from 8.615 mm to 1.551 mm, an 82.0% reduction

### 3. Five-Finger Contact Balancing

```python
imbalance = std(fingertip_forces) / max(mean(fingertip_forces) + 0.15, 0.15)
closure = 0.82 + min(0.14, 0.12 * imbalance)
```

- Five independent touch sensors measure contact across thumb, index, middle, ring, and little finger
- Contact imbalance increases grasp closure within a bounded range
- Ten finger joints coordinate around the keyed plug

### 4. Physical Proof-Load Validation

- A smooth external wrench peaks at 8.9987 N during the tug-test stage
- The controller continues sensor-driven corrections while the load is active
- Power verification occurs only after insertion, collar locking, and proof-load recovery

---

## 📊 Performance Metrics

| Metric | Value |
|---|---|
| Task Stages Completed | 9/9 |
| Terminal Conditions Passed | 6/6 |
| Task Completion | 100% |
| Minimum Insertion Error | 4.042 mm |
| Median Raw Servo Error | 8.615 mm |
| Median Corrected Servo Error | 1.551 mm |
| Median Error Reduction | 82.0% |
| Peak Residual Action | 23.420 mm |
| Closed-Loop Corrections | 29,698 |
| Peak Proof Load | 8.9987 N |
| Residual Evaluation Success | 100% (32/32) |
| Stage-Prior-Only Success | 0% (0/32) |
| Simulation Frequency | 500 Hz |

---

## 🛠️ Technical Specifications

### Robot Configuration

- **Robot actuators**: 16 channels
- **Positioning**: 3-DOF XYZ gantry
- **Wrist**: Yaw, pitch, and roll
- **Hand**: Five fingers with two independently actuated joints per finger
- **Task actuators**: Bayonet collar and guarded verification channel

### MuJoCo Model

- **Timestep**: 2 ms (500 Hz)
- **Total actuators**: 18
- **Sensors**: 13, including five fingertip touch channels
- **Robot joints**: 16 (three gantry, three wrist, and ten finger joints)
- **Joint types**: Free, slide, and hinge
- **Physics**: Frictional contacts, equality constraints, external wrench injection, and implicit integration
- **Assets**: Fully procedural MJCF with no external meshes or checkpoints

### Control Stack

- **Task planner**: Deterministic nine-stage state schedule
- **Position control**: Stage prior plus bounded visual-servo residual
- **Grasp control**: Contact-imbalance-driven finger closure
- **Fault injection**: Deterministic camera bias and proof-load disturbance
- **Evaluation**: 32 fixed-seed residual-controller ablation rollouts

---

## 📁 File Structure

```text
submissions/orbital_bayonet_repair/
├── run_orbital_repair.py       # Controller, simulation, rendering, and artifact generation
├── validate_submission.py     # Structural, metadata, artifact, and MuJoCo validation
├── orbital_bayonet_scene.xml  # Procedural hand and satellite connector scene
├── requirements.txt           # Python dependencies
├── README.md                  # This file
├── PR_DESCRIPTION.md          # Pull-request summary
├── evaluation_report.json     # Rubric-aligned evaluation summary
├── registration.json          # Robothon registration metadata
└── artifacts/
    ├── demo.mp4                 # Generated 60-second demonstration
    ├── trajectory.json          # Time-indexed states, actions, contacts, and labels
    ├── report.json              # Runtime metrics and terminal conditions
    ├── evaluation.json          # Fixed-seed controller ablation
    ├── policy_card.json         # Controller observations, actions, and scope
    ├── contact_timeline.json    # Fingertip force and proof-load timeline
    └── narration.srt            # Stage-aligned captions
```

---

## 🚀 Quick Start

Run from the repository root:

```bash
# Create an isolated environment and install dependencies
python3 -m venv .venv
.venv/bin/python -m pip install -r submissions/orbital_bayonet_repair/requirements.txt

# Generate the full demonstration and evidence package
.venv/bin/python submissions/orbital_bayonet_repair/run_orbital_repair.py

# Validate the submission
.venv/bin/python submissions/orbital_bayonet_repair/validate_submission.py
```

For a faster smoke test:

```bash
.venv/bin/python submissions/orbital_bayonet_repair/run_orbital_repair.py --quick
```

The runner uses EGL on headless Linux and falls back to a deterministic schematic renderer if no OpenGL context is available.

---

## 📈 Evaluation Results

The generated `artifacts/evaluation.json` contains 32 deterministic disturbance rollouts using seed `20260619`. Each rollout varies the initial pose, camera-calibration bias, proof-load impulse, and damping:

- The closed-loop residual controller succeeds in 32/32 cases (100%)
- The stage prior without residual feedback succeeds in 0/32 cases (0%)
- Closed-loop median final error is 1.332 mm
- Closed-loop 95th-percentile final error is 2.333 mm
- Stage-prior-only median final error is 45.287 mm

Runtime MuJoCo measurements and all six terminal conditions are recorded separately in `artifacts/report.json`.

---

## 🏆 Why it Stand out?

1. **Complete Repair Sequence**: Grasping, calibration recovery, insertion, locking, proof testing, and verification form one coherent task
2. **Measured Closed-Loop Value**: A paired ablation isolates the contribution of sensor-driven residual control
3. **Five-Finger Dexterity**: Ten finger joints and five touch channels coordinate around a keyed connector
4. **Physical Validation**: The repair must survive a measured external proof load before power verification
5. **Reproducible Evidence**: One deterministic runner generates video, trajectories, reports, evaluation data, policy disclosure, contact timelines, and captions

---

## 📝 License

This project is submitted for the FFAI Robothon 2026 competition.
