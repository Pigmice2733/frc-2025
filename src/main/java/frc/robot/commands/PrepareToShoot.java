package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class PrepareToShoot extends SequentialCommandGroup {
  public PrepareToShoot(Shooter shr, CommandXboxController ctlr) {
    addCommands(
        Commands.runOnce(() -> shr.setTargetFlywheelSpeed(ShooterConfig.FLYWHEEL_HIGH_SPEED)),
        Commands.waitUntil(() -> shr.flywheelsAtSpeed()),
        Commands.runOnce(() -> ctlr.setRumble(RumbleType.kBothRumble, 0.5)),
        Commands.waitSeconds(0.25),
        Commands.runOnce(() -> ctlr.setRumble(RumbleType.kBothRumble, 0)));
  }
}
