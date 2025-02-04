package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class SetElevatorPosition extends Command {
  private Pivot pivot;
  private Elevator elevator;
  private PIDController pivotController, elevatorController;
  private ElevatorPosition endPosition;

  /** Sets the elevator and pivot arm to the given position. */
  public SetElevatorPosition(Elevator elevator, Pivot pivot, ElevatorPosition position) {
    this.pivot = pivot;
    this.elevator = elevator;
    pivotController = pivot.getController();
    elevatorController = elevator.getController();
    endPosition = position;

    addRequirements(pivot, elevator);
  }

  @Override
  public void initialize() {
    pivotController.setSetpoint(endPosition.getPivotAngle());
    elevatorController.setSetpoint(endPosition.getElevatorHeight());
  }

  @Override
  public void execute() {
    pivot.setSpeed(pivotController.calculate(pivot.getAngle()));
    elevator.setSpeed(elevatorController.calculate(elevator.getHeight()));
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
    elevator.setSpeed(0);
    RobotContainer.setElevatorPosition(endPosition);
  }

  @Override
  public boolean isFinished() {
    return pivotController.atSetpoint() && elevatorController.atSetpoint();
  }
}
