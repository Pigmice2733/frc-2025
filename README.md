# frc-2025

Robot code to play the 2025 Reefscape FRC game.

## Subsystems

- swerve drivetrain (see YAGSL dependency) with slowmode option
- elevator has two motors moving together to raise other subsystems
- pivot at top of elevator (moves algae and coral manipulators) has one motor with absolute encoder
- algae intake has one motor
- coral intake has two motors moving together but opposite directions
- shooter (lowest priority) has one or two motors for the pivot-base (together), one for the indexer, and two for the flywheels (together)

## Controls

driver: left joysticks for driving, right joystick for turning, A for reset, Y to toggle slowmode, B to auto-align
operator: check in with strategy later, probably d-pad for coral scoring

## Automation

- software stops on elevator pivot
- intake algae and score coral
- intake coral from station with vision alignment
- score at processor with vision alignment
- reset everything
