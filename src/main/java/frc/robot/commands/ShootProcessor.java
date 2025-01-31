package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.AlgaeShooter;

public class ShootProcessor extends SequentialCommandGroup {
  /** Scores an algae in the processor. */
  public ShootProcessor(AlgaeShooter shooter) {
    addCommands(
        new InstantCommand(() -> shooter.setFlywheels(SystemConfig.FLYWHEEL_PROCESSOR_SPEED)),
        new WaitCommand(SystemConfig.SHOOTER_SPINUP_TIME),
        shooter.runIndexerForward(),
        new WaitCommand(SystemConfig.SHOOTER_SHOOT_TIME),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}