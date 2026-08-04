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


def resolve_actual_topic(
    base_name: str,
    graph_types: Dict[str, List[str]],
) -> str:
    """Return the topic name that actually exists in the ROS graph.

    uXRCE-DDS appends ``_vN`` to a topic when the underlying px4_msgs message
    declares a non-zero ``MESSAGE_VERSION`` constant. The contract is written
    in the unversioned form (matching ``dds_topics.yaml``) so this helper maps
    the required name to the live topic at audit time.

    Falls back to the base name when no versioned variant is found, which is
    the correct behaviour for messages whose version is 0 (e.g.
    ``VehicleOdometry``) and for non-px4 topics.
    """
    if base_name in graph_types:
        return base_name
    prefix = base_name + '_v'
    for topic in sorted(graph_types.keys()):
        if topic.startswith(prefix):
            return topic
    return base_name


def evaluate_interface(
    required_topics: Iterable[str],
    graph_types: Dict[str, List[str]],
    publisher_nodes: Dict[str, List[str]],
    subscriber_nodes: Dict[str, List[str]],
    require_fmu_writer: bool = True,
) -> dict:
    """Return the safety-relevant summary used by the runtime graph audit."""
    required = list(required_topics)
    resolved = {
        name: resolve_actual_topic(name, graph_types) for name in required
    }

    def _lookup(nodes: Dict[str, List[str]], name: str) -> List[str]:
        """Return the node list keyed by resolved or base name.

        Callers (and the audit node) key the node maps by the topic name they
        see in the ROS graph. Resolution may rewrite that name (e.g. add a
        ``_v1`` suffix), so look up both forms and prefer the resolved one.
        """
        actual = resolved.get(name, name)
        if actual in nodes:
            return nodes[actual]
        if name in nodes:
            return nodes[name]
        return []


    missing = [name for name in required if resolved[name] not in graph_types]
    unpublished = [
        name for name in required
        if not _lookup(publisher_nodes, resolved[name])
    ]
    disconnected_commands = [
        name for name in required
        if resolved[name] in COMMAND_TOPICS and
        not _lookup(subscriber_nodes, resolved[name])
    ]
    invalid_writers: Dict[str, List[str]] = {}
    if require_fmu_writer:
        for name in FMU_INPUT_TOPICS:
            writers = _lookup(publisher_nodes, name)
            if writers != ['/trajectory_executor']:
                invalid_writers[name] = writers
    return {
        'ok': (
            not missing and not unpublished and
            not disconnected_commands and not invalid_writers
        ),
        'missing': missing,
        'unpublished': unpublished,
        'disconnected_commands': disconnected_commands,
        'unique_fmu_writer': not invalid_writers,
        'invalid_fmu_writers': invalid_writers,
        'resolved_topics': resolved,
    }


def direct_rotor_output_allowed(enabled: bool, backend_mode: str) -> bool:
    """Require two independent arming gates before raw rotor output is legal."""
    return enabled and backend_mode == 'direct_rotor'


def observed_frequency_hz(sample_times: List[float]) -> float:
    """Estimate topic frequency from monotonic receive timestamps."""
    if len(sample_times) < 2 or sample_times[-1] <= sample_times[0]:
        return 0.0
    return (len(sample_times) - 1) / (sample_times[-1] - sample_times[0])
