# 2.3 Extension to Real Robot

- The command topics to the robot are abstracted
  - Next steps I should predict the wheel speed not the yaw and speed
  - Update model accurate for MPC
- Integration with path planning & Hardware
  - Path planning -> Path tracking
  - Path tracking -> STATEMACHINE -> CAN
- Handle odom edge cases
- Testing
  - Real world testing
  - Tuning
  - More simulation testing
- Missing things for production ready
  - No PR. Due to time constrain the names might be not intutive

# 2.4 AI Tools Used

- GitHub Copilot: Autocomplete
- Claude: Final documentation

The work ouput from the tools are verified  

# 2.5 Obstacle Avoidance Extension

- Hierarchical planners
  - Global planner can re-plan in case of obstacle
    - Existing planners can do that - Hybrid A*/RRT*
    - You can add hueristics, Example:
      - use obstacle as potential field and apply path smoothing
- MPC
  - Add obstacles as hard constrain for MPC