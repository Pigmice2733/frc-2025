package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Elevator;

public class ElevatorControl extends Command {
  private Elevator elevator;
  private DoubleSupplier speed;

  /** Adjusts the elevator's set position based on joystick input. */
  public ElevatorControl(Elevator elevator, DoubleSupplier joystickSpeed) {
    this.elevator = elevator;
    speed = joystickSpeed;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    // System.out.println(
    // "Elevator pos: " + elevator.getHeight() + " | Setting to: " +
    // (elevator.getHeight() + speed.getAsDouble()));
    elevator.changeSetpoint(speed.getAsDouble());
    if (speed.getAsDouble() != 0) {
      RobotContainer.setElevatorPosition(ArmPosition.STOW);
    }
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
