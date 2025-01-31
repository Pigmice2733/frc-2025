package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotPosition;
import frc.robot.subsystems.Pivot;

public class SetPivotPosition extends Command {
  private Pivot pivot;
  private PIDController motorController;
  private double goal;

  /** Sets the pivot to the given position. */
  public SetPivotPosition(Pivot pivot, PivotPosition position) {
    this.pivot = pivot;
    motorController = pivot.getController();
    goal = position.getAngle();

    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    motorController.setSetpoint(goal);
  }

  @Override
  public void execute() {
    pivot.setSpeed(motorController.calculate(pivot.getAngle()));
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return motorController.atSetpoint();
  }
}
