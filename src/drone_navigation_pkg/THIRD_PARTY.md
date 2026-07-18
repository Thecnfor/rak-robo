# Third-party algorithm provenance

The local planner follows the ESDF-free replanning architecture described by
EGO-Planner. The requested integration baseline is commit
`23a8d5a191711dd65633df689bd00f55d4dea8f9` from
<https://github.com/ZJU-FAST-Lab/ego-planner> (GPL-3.0).

That object is not advertised by the official repository and a direct fetch
failed on 2026-07-18. Consequently, no upstream source file is currently
vendored and this package does not claim source-level compatibility with that
commit. The exact EGO/LBFGS integration ticket remains open until the owner of
the referenced ROS 2 fork or a reachable commit is supplied.

This package does not deploy the upstream ROS graph. Its current rolling voxel
map, line collision checks, dynamic A*, uniform B-spline sampling, and
feasibility correction are an independent interim implementation isolated
behind the `VoxelPlanner` and `UniformBsplineTrajectory` public interfaces.
