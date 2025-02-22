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
    endPosition = position;

    addRequirements(pivot, elevator);
  }

  @Override
  public void initialize() {
    pivotController = pivot.getController();
    elevatorController = elevator.getController();

    System.out.println(
        "Pivot: P=" + pivotController.getP() + " I=" + pivotController.getI() + " D=" + pivotController.getD());
    System.out.println("Elevator: P=" + elevatorController.getP() + " I=" + elevatorController.getI() + " D="
        + elevatorController.getD());

    pivotController.setSetpoint(endPosition.getPivotAngle());
    elevatorController.setSetpoint(endPosition.getElevatorHeight());
  }

  @Override
  public void execute() {
    pivot.setSpeed(pivotController.calculate(pivot.getAngle()));
    elevator.setSpeeds(elevatorController.calculate(elevator.getHeight()));
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
    elevator.setSpeeds(0);
    RobotContainer.setElevatorPosition(endPosition);
  }

  @Override
  public boolean isFinished() {
    return pivotController.atSetpoint() && elevatorController.atSetpoint();
  }
}
