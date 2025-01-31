package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.AlgaeShooter;

public class ShootAlgae extends SequentialCommandGroup {
  /** Shoots an algae. */
  public ShootAlgae(AlgaeShooter shooter) {
    addCommands(
        shooter.runFlywheelsForward(),
        new WaitCommand(SystemConfig.SHOOTER_SPINUP_TIME),
        shooter.runIndexerForward(),
        new WaitCommand(SystemConfig.SHOOTER_SHOOT_TIME),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}
