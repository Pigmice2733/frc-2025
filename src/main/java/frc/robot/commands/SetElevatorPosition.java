package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class SetElevatorPosition extends Command {
  private Pivot pivot;
  private Elevator elevator;
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
    // System.out.println(
    // "Pivot: P=" + pivotController.getP() + " I=" + pivotController.getI() + " D="
    // + pivotController.getD());
    // System.out.println("Elevator: P=" + elevator.getP() + " I=" +
    // elevatorController.getI() + " D="
    // + elevatorController.getD());

    pivot.setSetpoint(endPosition.getPivotAngle());
    elevator.setSetpoint(endPosition.getElevatorHeight());
    System.out.println(
        "Setting Setpoint to Elev: " + endPosition.getElevatorHeight() + ", Pivot: " + endPosition.getPivotAngle());
  }

  @Override
  public void execute() {
    // System.out.println("pivot.getanlge: " + pivot.getAngle());
    // System.out.println("Setting Pivot Speed to: " + pivotSpeed);
    pivot.setSpeed(pivot.calculate());
    elevator.setSpeeds(elevator.calculate());
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
    elevator.setSpeeds(0);
    RobotContainer.setElevatorPosition(endPosition);
  }

  @Override
  public boolean isFinished() {
    return pivot.atSetpoint() && elevator.atSetpoint();
  }
}
