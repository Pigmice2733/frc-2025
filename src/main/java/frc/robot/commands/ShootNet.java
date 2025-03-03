package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.Shooter;

public class ShootNet extends SequentialCommandGroup {
  /** Scores an algae in the net. */
  public ShootNet(Shooter shooter) {
    addCommands(
        new InstantCommand(() -> shooter.setFlywheels(ShooterConfig.FLYWHEEL_NET_SPEED)),
        new WaitCommand(ShooterConfig.SPINUP_TIME),
        shooter.runIndexerForward(),
        new WaitCommand(ShooterConfig.SHOOT_TIME),
        shooter.stopMotors());
    addRequirements(shooter);
  }
}
