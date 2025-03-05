package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class ShootProcessor extends SequentialCommandGroup {
  /** Scores an algae in the processor. */
  public ShootProcessor(Shooter shooter) {
    // addCommands(
    // new InstantCommand(() ->
    // shooter.setFlywheels(ShooterConfig.FLYWHEEL_PROCESSOR_SPEED)),
    // Commands.waitUntil(() -> shooter.flywheelsAtProcessorSpeed()),
    // shooter.runIndexerForward(),
    // Commands.waitUntil(() -> !shooter.hasAlgae()),
    // shooter.stopMotors());
    // addRequirements(shooter);
  }
}