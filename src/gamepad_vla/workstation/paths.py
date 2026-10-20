"""Resolve workspace asset references without changing scene geometry."""
from gamepad_vla.site import workspace_root


def resolve_workspace_path(value):
    """Leave ordinary and historical paths intact; resolve workspace:// paths."""
    if value.startswith("workspace://"):
        return str(workspace_root() / value.removeprefix("workspace://"))
    return value
