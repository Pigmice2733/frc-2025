package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.AlgaeShooter;

public class AlgaeFromReef extends SequentialCommandGroup {
  /** Collects an algae from the reef. */
  public AlgaeFromReef(AlgaeShooter shooter) {
    addCommands(
        shooter.runFlywheelsReverse(),
        shooter.runIndexerReverse(),
        new WaitCommand(SystemConfig.SHOOTER_INTAKE_TIME),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}