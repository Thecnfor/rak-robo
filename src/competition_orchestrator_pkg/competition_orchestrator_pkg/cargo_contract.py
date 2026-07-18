"""Canonical cargo-bay command and acknowledgement strings."""


def cargo_command_and_expected(door: str, action: str) -> tuple[str, str]:
    if door not in {'left', 'bottom'}:
        raise ValueError(f'unsupported cargo door: {door}')
    if action not in {'open', 'close'}:
        raise ValueError(f'unsupported cargo action: {action}')
    command = f'{door}_{action}'
    expected = f'{door}_{"opened" if action == "open" else "closed"}'
    return command, expected
