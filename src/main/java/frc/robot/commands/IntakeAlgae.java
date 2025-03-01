package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class IntakeAlgae extends SequentialCommandGroup {
  /** Intakes an algae from the ground. */
  public IntakeAlgae(Shooter shooter) {
    addCommands(
        new InstantCommand(() -> shooter.setFlywheels(-ShooterConfig.FLYWHEEL_PROCESSOR_SPEED)),
        shooter.runIndexerReverse(),
        new WaitCommand(ShooterConfig.INTAKE_TIME),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}
