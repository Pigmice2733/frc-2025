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

On the driver controller, the left joystick is for driving, the right joystick is for turning, A resets the pose, Y toggles slowmode, and the d-pad buttons auto-align to various tags.

The operator controls are mostly divided into three modes:

- In coral mode, the four d-pad buttons move the elevator to the four reef levels, and the right bumper outtakes coral from the manipulator. X enters coral mode.
- In elevator mode, two d-pad buttons move the elevator to positions for removing algae from the reef, one to the position for intaking from the station, and one to all the way down. The right bumper activates the algae grabber at the algae positions, intakes coral to the manipulator at the station position, and does nothing at the last position. B enters elevator mode.
- In shooter mode, one d-pad button moves the algae shooter to the intaking position, one to the position for scoring in the processor, and one to the position for shooting into the net. The fourth does nothing. The left bumper intakes algae at the intaking position and shoots at the shooting positions. A enters shooter mode.

Operator Y activates the climber.

## Automation

- software stops on elevator pivot
- intake algae from ground
- intake coral from station with vision alignment
- score algae at processor and at net with vision alignment
- score coral at all reef positions
- displace algae from reef
- reset everything
