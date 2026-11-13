"""Helpers shared by the autonom_bringup and robot_sim launch files."""

from typing import List

from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def is_true(context: LaunchContext, argument: str) -> bool:
    return LaunchConfiguration(argument).perform(context).strip().lower() in ('true', '1', 'yes', 'on')


def parameter_files(context: LaunchContext, base, overlay_argument: str) -> List:
    """Return [base] plus the overlays named by `overlay_argument`: empty, one file, or several separated by
    commas (later files win)."""
    overlays = LaunchConfiguration(overlay_argument).perform(context).split(',')
    return [base] + [overlay.strip() for overlay in overlays if overlay.strip()]
