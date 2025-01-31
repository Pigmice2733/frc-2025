package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.Shooter;

public class ShootAlgae extends SequentialCommandGroup {
  public ShootAlgae(Shooter shooter) {
    addCommands(
        shooter.runFlywheels(),
        new WaitCommand(SystemConfig.SHOOTER_SPINUP_TIME),
        shooter.runIndexer(),
        new WaitCommand(SystemConfig.SHOOTER_SHOOT_TIME),
        shooter.stopMotors());
  }

}
