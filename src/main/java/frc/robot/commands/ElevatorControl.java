package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class ElevatorControl extends Command {
  private Elevator elevator;
  private PIDController elevatorController;
  private DoubleSupplier speed;

  /** Sets the elevator and pivot arm to the given position. */
  public ElevatorControl(Elevator elevator, DoubleSupplier joystickSpeed) {
    this.elevator = elevator;
    speed = joystickSpeed;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevatorController = elevator.getController();
  }

  @Override
  public void execute() {
    elevatorController.setSetpoint(elevator.getHeight() + speed.getAsDouble() * 0.1);
    elevator.setSpeeds(elevatorController.calculate(elevator.getHeight()));
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setSpeeds(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
