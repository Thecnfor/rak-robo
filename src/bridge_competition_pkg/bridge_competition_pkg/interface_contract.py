"""Pure helpers for validating the Isaac/PX4 ROS graph contract."""

from typing import Dict, Iterable, List


FMU_INPUT_TOPICS = (
    '/fmu/in/offboard_control_mode',
    '/fmu/in/trajectory_setpoint',
    '/fmu/in/vehicle_command',
)
COMMAND_TOPICS = (
    *FMU_INPUT_TOPICS,
    '/cargo_bay/command',
    '/drone0/control/rotor0/ref',
    '/drone0/control/rotor1/ref',
    '/drone0/control/rotor2/ref',
    '/drone0/control/rotor3/ref',
)


def evaluate_interface(
    required_topics: Iterable[str],
    graph_types: Dict[str, List[str]],
    publisher_nodes: Dict[str, List[str]],
    subscriber_nodes: Dict[str, List[str]],
    require_fmu_writer: bool = True,
) -> dict:
    """Return the safety-relevant summary used by the runtime graph audit."""
    required = list(required_topics)
    missing = [name for name in required if name not in graph_types]
    unpublished = [name for name in required if not publisher_nodes.get(name)]
    disconnected_commands = [
        name for name in required
        if name in COMMAND_TOPICS and not subscriber_nodes.get(name)
    ]
    multiple_writers = [
        name for name in FMU_INPUT_TOPICS
        if len(publisher_nodes.get(name, [])) > 1
    ]
    invalid_writers = {}
    if require_fmu_writer:
        invalid_writers = {
            name: publisher_nodes.get(name, [])
            for name in FMU_INPUT_TOPICS
            if publisher_nodes.get(name, []) != ['/trajectory_executor']
        }
    return {
        'ok': (
            not missing and not unpublished and
            not disconnected_commands and not invalid_writers
        ),
        'missing': missing,
        'unpublished': unpublished,
        'disconnected_commands': disconnected_commands,
        'unique_fmu_writer': not invalid_writers,
        'multiple_fmu_writers': multiple_writers,
        'invalid_fmu_writers': invalid_writers,
    }


def direct_rotor_output_allowed(enabled: bool, backend_mode: str) -> bool:
    """Require two independent arming gates before raw rotor output is legal."""
    return enabled and backend_mode == 'direct_rotor'


def observed_frequency_hz(sample_times: List[float]) -> float:
    """Estimate topic frequency from monotonic receive timestamps."""
    if len(sample_times) < 2 or sample_times[-1] <= sample_times[0]:
        return 0.0
    return (len(sample_times) - 1) / (sample_times[-1] - sample_times[0])
