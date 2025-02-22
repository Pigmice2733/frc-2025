package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class TestDrive extends SequentialCommandGroup {
  /** Drives forward, then left, then turns left, for the purpose of testing. */
  public TestDrive(Drivetrain dvt) {
    addCommands(
        dvt.reset(),
        Commands.deadline(new WaitCommand(0.5),
            dvt.driveCommand(DrivetrainConfig.MAX_DRIVE_SPEED, 0, 0)),
        Commands.deadline(new WaitCommand(0.5),
            dvt.driveCommand(0, DrivetrainConfig.MAX_DRIVE_SPEED, 0)),
        Commands.deadline(new WaitCommand(0.5),
            dvt.driveCommand(0, 0, DrivetrainConfig.MAX_TURN_SPEED)),
        dvt.driveCommand(0, 0, 0));
    addRequirements(dvt);
  }
}
