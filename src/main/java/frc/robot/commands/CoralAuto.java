package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.*;

public class CoralAuto extends SequentialCommandGroup {
  public CoralAuto(Drivetrain dvt, Vision vis, Elevator elv, Pivot pivot, Grabber gbr, CommandXboxController ctlr) {
    addCommands(
        dvt.driveCommand(DrivetrainConfig.MAX_DRIVE_SPEED, 0, 0),
        Commands.waitUntil(() -> (dvt.getPose().getX() > 2)),
        dvt.driveCommand(0, 0, DrivetrainConfig.MAX_TURN_SPEED),
        Commands.waitUntil(() -> (dvt.getPose().getRotation().getDegrees() > 180)),
        new DriveToTarget(dvt, vis, ctlr, Units.inchesToMeters(17), Units.inchesToMeters(6.5), 0),
        new SetArmPosition(elv, pivot, ArmPosition.ALGAE_L3),
        gbr.runForward(),
        Commands.waitSeconds(2),
        gbr.stopMotor(),
        new SetArmPosition(elv, pivot, ArmPosition.SCORE_L3),
        gbr.runReverse(),
        Commands.waitSeconds(2),
        gbr.stopMotor(),
        new SetArmPosition(elv, pivot, ArmPosition.STOW));
  }
}
