package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

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
    System.out.println(
        "Elevator pos: " + elevator.getHeight() + " | Setting to: " + (elevator.getHeight() + speed.getAsDouble() * 1));
    elevatorController.setSetpoint(elevator.getHeight() + speed.getAsDouble() * 1);
    // SetElevatorPosition(elevator.getHeight() + speed.getAsDouble() * 1);
    elevator.setMotorSpeed(elevatorController.calculate(elevator.getHeight()));
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
