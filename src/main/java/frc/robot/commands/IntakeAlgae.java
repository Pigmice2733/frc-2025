package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class IntakeAlgae extends SequentialCommandGroup {
  /** Intakes an algae from the ground. */
  public IntakeAlgae(Shooter shooter) {
    addCommands(
        shooter.runIndexerIn(),
        Commands.runOnce(() -> shooter.setTargetFlywheelSpeed(-ShooterConfig.FLYWHEEL_LOW_SPEED), shooter)
    // Commands.waitUntil(() -> shooter.hasAlgae()),
    // shooter.stopMotors()
    );
    addRequirements(shooter);
  }
}
