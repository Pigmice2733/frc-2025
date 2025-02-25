package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotControl extends Command {
  private Pivot pivot;
  private DoubleSupplier speed;

  /** Adjusts the pivot's set position based on joystick input. */
  public PivotControl(Pivot pivot, DoubleSupplier joystickSpeed) {
    this.pivot = pivot;
    speed = joystickSpeed;

    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.changeSetpoint(speed.getAsDouble() * 0.1);
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
