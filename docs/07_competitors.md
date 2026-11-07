# Simulated Competitors

To properly test dynamic obstacle avoidance, the simulation must include competitor robots as defined by the competition rules.

## Competitor Specifications
*   **Dimensions:** 15cm x 15cm (Standard AC rules max bounding box).
*   **Quantity:** 2 competitor robots.
*   **Model:** Simple box geometry in Gazebo Harmonic with basic kinematics (differential drive).
*   **Behavior:** They should execute a pre-programmed path (e.g., using a simple ROS 2 node or Gazebo plugin) to continuously loop around the track.

## Placement & Start Logic
*   **Placement:** All robots (the main robot + 2 competitors) must be placed at the starting line.
*   **Alignment:** 
    *   According to rule AC-2.2 and AC-3.6, robots cannot be aligned in a way that intentionally rams the opponent at the start.
    *   Our main robot must be placed in the **middle** position.
    *   Competitor 1 should be placed to the left.
    *   Competitor 2 should be placed to the right.
*   **Start Delay:** The competitors should ideally respect the same 5-second start delay (AC-3.2, AC-4.5) to accurately simulate the race start conditions.
