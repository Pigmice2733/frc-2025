package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotControl extends Command {
  private Pivot pivot;
  private PIDController pivotController;
  private DoubleSupplier speed;

  /** Sets the elevator and pivot arm to the given position. */
  public PivotControl(Pivot pivot, DoubleSupplier joystickSpeed) {
    this.pivot = pivot;
    speed = joystickSpeed;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivotController = pivot.getController();
  }

  @Override
  public void execute() {
    pivotController.setSetpoint(pivot.getAngle() + speed.getAsDouble() * 0.1);
    pivot.setSpeed(pivotController.calculate(pivot.getAngle()));
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
