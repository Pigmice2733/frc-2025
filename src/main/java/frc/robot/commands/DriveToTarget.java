package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveToTarget extends SequentialCommandGroup {
  /**
   * Drives the robot to the given position with respect to its nearest target.
   * 
   * @param drivetrain drivetrain subsystem
   * @param vision     vision subsystem
   * @param xOffset    target distance from tag in the X direction (m)
   * @param yOffset    target distance from tag in the Y direction (m)
   * @param rOffset    target angle offset from tag (deg)
   */
  public DriveToTarget(Drivetrain drivetrain, Vision vision, CommandXboxController ctlr, double xOffset, double yOffset,
      double rOffset) {
    addCommands(
        Commands.runOnce(() -> {
          if (!vision.hasTarget())
            ctlr.setRumble(RumbleType.kBothRumble, 0.5);
        }),
        new TurnVision(drivetrain, vision, rOffset).withTimeout(2),
        new DriveVision(drivetrain, vision, xOffset, yOffset));
    addRequirements(drivetrain, vision);
  }
}
