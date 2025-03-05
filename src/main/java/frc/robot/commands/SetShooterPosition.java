package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class SetShooterPosition extends Command {
  private Shooter shooter;
  private PIDController controller;
  private ShooterPosition endPosition;

  /** Sets the shooter to the given position. */
  public SetShooterPosition(Shooter shooter, ShooterPosition position) {
    this.shooter = shooter;
    controller = shooter.getController();
    endPosition = position;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    controller.setSetpoint(endPosition.getAngle());
  }

  @Override
  public void execute() {
    shooter.setPivot(controller.calculate(shooter.getPivotPosition()));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPivot(0);
    RobotContainer.setShooterPosition(endPosition);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
