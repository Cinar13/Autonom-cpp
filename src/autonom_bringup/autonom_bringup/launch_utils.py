"""Helpers shared by the autonom_bringup and robot_sim launch files."""

from typing import List

from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def is_true(context: LaunchContext, argument: str) -> bool:
    return LaunchConfiguration(argument).perform(context).strip().lower() in ('true', '1', 'yes', 'on')


def parameter_files(context: LaunchContext, base, overlay_argument: str) -> List:
    """Return [base] plus the overlay named by `overlay_argument` when that argument is non-empty."""
    overlay = LaunchConfiguration(overlay_argument).perform(context).strip()
    return [base, overlay] if overlay else [base]
