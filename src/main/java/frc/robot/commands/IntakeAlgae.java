package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.AlgaeShooter;

public class IntakeAlgae extends SequentialCommandGroup {
  /** Intakes an algae from the ground. */
  public IntakeAlgae(AlgaeShooter shooter) {
    addCommands(
        new InstantCommand(() -> shooter.setFlywheels(-SystemConfig.FLYWHEEL_PROCESSOR_SPEED)),
        shooter.runIndexerReverse(),
        new WaitCommand(SystemConfig.SHOOTER_INTAKE_TIME),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}
