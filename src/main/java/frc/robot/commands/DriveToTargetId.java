package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveToTargetId extends SequentialCommandGroup {

  /**
   * Drives the robot to the given position with respect the provided target.
   * 
   * @param target     target id
   * @param drivetrain drivetrain subsystem
   * @param vision     vision subsystem
   * @param ctlr       Xbox controller
   * @param xOffset    target distance from tag in the X direction (m)
   * @param yOffset    target distance from tag in the Y direction (m)
   * @param rOffset    target angle offset from tag (deg)
   */
  public DriveToTargetId(int target, Drivetrain drivetrain, Vision vision, CommandXboxController ctlr, double xOffset,
      double yOffset, double rOffset) {
    addCommands(
        new TurnUntilTargetId(target, 5.0, vision, drivetrain),
        new DriveToTarget(drivetrain, vision, ctlr, xOffset, yOffset, rOffset));

    addRequirements(drivetrain, vision);
  }
}