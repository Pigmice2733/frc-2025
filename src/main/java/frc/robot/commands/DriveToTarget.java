package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveToTarget extends SequentialCommandGroup {
  private Pose2d initialPose;

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
        Commands.runOnce(() -> initialPose = drivetrain.getPose()),
        new TurnVision(drivetrain, vision, rOffset),
        new DriveVision(drivetrain, vision, xOffset, yOffset),
        Commands.runOnce(() -> drivetrain.resetPose(initialPose)));
    addRequirements(drivetrain, vision);
  }
}
