package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class SetArmPosition extends Command {
  private Pivot pivot;
  private Elevator elevator;
  private ArmPosition endPosition;

  /** Sets the elevator and pivot arm to the given position. */
  public SetArmPosition(Elevator elevator, Pivot pivot, ArmPosition position) {
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
    // System.out.println(
    // "Setting Setpoint to Elev: " + endPosition.getElevatorHeight() + ", Pivot: "
    // + endPosition.getPivotAngle());
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
    elevator.setSpeeds(0);
    RobotContainer.setArmPosition(endPosition);
  }

  @Override
  public boolean isFinished() {
    return pivot.atSetpoint() && elevator.atSetpoint();
  }
}
