# Autonomous Car Competition Constraints

This project is governed by the Fibonacci International Robot Olympiad - Autonomous Car (AC) rules. The AI agent must strictly enforce the following logic and parameters in the ROS 2 software stack:

## 1. Mandatory 5-Second Start Delay (AC-3.2, AC-4.5)
*   **Rule:** After the operator triggers the "start" command (via a physical button or RC5 IR remote), the robot **must wait exactly 5 seconds** before initiating any movement or autonomous navigation.
*   **Software Requirement:** 
    *   Create a "Lifecycle / State Machine Node" that intercepts the start signal.
    *   This node must hold the robot in a `WAITING` state, block `cmd_vel` output, and prevent `Nav2` from sending goals until a 5-second timer expires.
    *   Moving before 5 seconds results in an immediate penalty (-1 point).

## 2. Dynamic Obstacles & Wall Avoidance (AC-3.5, AC-4.8)
*   **Rule (Wall Tracking Ban):** The robot cannot continuously touch, slide against, or use the walls as a physical guide. Unintentional wall contact is permitted but must not exceed 3 seconds.
*   **Rule (Dynamic Navigation):** There will be 2 to 4 robots operating on the closed-loop track simultaneously. The robot must not intentionally ram or use other robots for navigation.
*   **Software Requirement:**
    *   The robot must rely entirely on LiDAR (`sllidar_ros2`) and `Nav2` local costmaps to avoid walls and moving competitor robots.
    *   Configure the local trajectory planner (e.g., DWB or MPPI) with high inflation radius penalties near walls to ensure the robot stays centered on the track.
    *   Implement recovery behaviors to back away cleanly if the LiDAR detects a collision or if the robot gets stuck, minimizing the risk of exceeding the 3-second contact rule.

## 3. Autonomous Looping & Scoring (AC-5.1)
*   **Rule:** The robot scores +1 point every time it completes a full lap in the designated direction.
*   **Software Requirement:**
    *   The `slam_toolbox` node must map the closed loop asynchronously.
    *   A continuous looping script (Python action client) should feed sequential Nav2 waypoints along the track to keep the robot circling endlessly for the 3-minute round duration.
