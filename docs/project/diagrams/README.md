# State-machine diagrams

Render with any Mermaid tool, e.g.:

```bash
# 1) Mermaid CLI:
npm install -g @mermaid-js/mermaid-cli
mmdc -i mission_gate.mmd -o mission_gate.png
mmdc -i flight_supervisor.mmd -o flight_supervisor.png
mmdc -i grasp_pipeline.mmd -o grasp_pipeline.png
mmdc -i drone_chain.mmd -o drone_chain.png

# 2) GitHub / docs site: just commit the .mmd files; Mermaid renders them
# automatically on the GitHub web UI and on readthedocs / mkdocs with the
# `mermaid2` extension.
```

| File | Subject | Code source of truth |
|---|---|---|
| `mission_gate.mmd` | `competition_orchestrator_pkg/mission_gate.py::MissionGate.observe` — single-shot dispatch after `COMPLETE` | `src/competition_orchestrator_pkg/competition_orchestrator_pkg/mission_gate.py` |
| `flight_supervisor.mmd` | `drone_navigation_pkg::FlightSupervisor::update` — 11-phase mission + HOLD_EMERGENCY safety gates | `src/drone_navigation_pkg/src/flight_core.cpp` (FSM block) |
| `grasp_pipeline.mmd` | arm perception → plan-to-pose → gripper flow per `grasp_demo_pkg/config/demo_params.yaml` | `src/grasp_demo_pkg/launch/perception_pipeline_demo.launch.py` |
| `drone_chain.mmd` | PX4 SITL ↔ Isaac Sim ↔ host chain + sole-writer rule on `/fmu/in/*` | `src/drone_navigation_pkg/launch/navigation.launch.py` |
