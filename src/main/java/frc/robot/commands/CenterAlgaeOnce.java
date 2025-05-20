package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class CenterAlgaeOnce extends SequentialCommandGroup {
  /** Intakes an algae from the ground. */
  public CenterAlgaeOnce(Shooter shooter) {
    addCommands(
        Commands.runOnce(() -> shooter.setTargetFlywheelSpeed(ShooterConfig.FLYWHEEL_LOW_SPEED), shooter),
        shooter.runIndexerOut(),
        Commands.waitSeconds(0.2),
        shooter.runIndexerIn(),
        Commands.waitUntil(() -> shooter.hasAlgae()).withTimeout(1.0),
        Commands.runOnce(() -> shooter.setTargetFlywheelSpeed(0.0), shooter));
    addRequirements(shooter);
  }
}
