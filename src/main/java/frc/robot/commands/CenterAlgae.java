package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class CenterAlgae extends SequentialCommandGroup {
  /** Intakes an algae from the ground. */
  public CenterAlgae(Shooter shooter) {
    addCommands(
        Commands.runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_PROCESSOR_ANGLE)),
        Commands.waitUntil(() -> shooter.pivotAtSetpoint()),
        new CenterAlgaeOnce(shooter).repeatedly()
            .until(() -> shooter.isAlgaeCentered() || shooter.isLowerAlgaeLimitSwitchPressed()).withTimeout(5.0),
        Commands.runOnce(() -> shooter.stopMotors()));
    addRequirements(shooter);
  }
}
