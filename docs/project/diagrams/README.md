# State-machine diagrams

Each diagram exists in two source formats:

- `.dot` — graphviz `dot` language; rendered to PNG by `dot -Tpng`.
- `.mmd` — mermaid; rendered by `mmdc` or by GitHub's Mermaid block
  when the file is committed.

The committed `.png` files are the ones embedded in
`docs/project/技术文档-XX队-预选赛.pdf`. Re-render when the source changes:

```bash
# PNG via Graphviz (offline, fast, already installed):
cd docs/project/diagrams
for f in mission_gate flight_supervisor grasp_pipeline drone_chain; do
  dot -Tpng "$f.dot" -o "$f.png"
done

# Mermaid source files are kept so the GitHub README renders them
# automatically with a ` ```mermaid ` block. No PNG needed from them.
```

| File | Subject | Code source of truth |
|---|---|---|
| `mission_gate.dot` + `.mmd` | `competition_orchestrator_pkg/mission_gate.py::MissionGate.observe` — single-shot dispatch after `COMPLETE` | `src/competition_orchestrator_pkg/competition_orchestrator_pkg/mission_gate.py` |
| `flight_supervisor.dot` + `.mmd` | `drone_navigation_pkg::FlightSupervisor::update` — 11-phase mission + HOLD_EMERGENCY safety gates | `src/drone_navigation_pkg/src/flight_core.cpp` (FSM block) |
| `grasp_pipeline.dot` + `.mmd` | arm perception → plan-to-pose → gripper flow per `grasp_demo_pkg/config/demo_params.yaml` | `src/grasp_demo_pkg/launch/perception_pipeline_demo.launch.py` |
| `drone_chain.dot` + `.mmd` | PX4 SITL ↔ Isaac Sim ↔ host chain + sole-writer rule on `/fmu/in/*` | `src/drone_navigation_pkg/launch/navigation.launch.py` |
