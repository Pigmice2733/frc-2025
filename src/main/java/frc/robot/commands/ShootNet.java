package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShootNet extends SequentialCommandGroup {
  /** Scores an algae in the net. */
  public ShootNet(Shooter shooter) {
    addCommands(
        shooter.runIndexerOut(),
        Commands.waitUntil(() -> !shooter.hasAlgae()),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}
