package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class ShootNet extends SequentialCommandGroup {
  /** Scores an algae in the net. */
  public ShootNet(Shooter shooter, CommandXboxController ctlr) {
    addCommands(
        shooter.runIndexerOut(),
        Commands.runOnce(() -> ctlr.setRumble(RumbleType.kBothRumble, 0))
    // Commands.waitUntil(() -> !shooter.hasAlgae()),
    // shooter.stopMotors()
    );
    addRequirements(shooter);
  }
}
