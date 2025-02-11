package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveJoysticks extends Command {
  private Drivetrain dvt;
  private DoubleSupplier x, y, r;

  public DriveJoysticks(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turnSpeed) {
    dvt = drivetrain;
    x = xSpeed;
    y = ySpeed;
    r = turnSpeed;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    dvt.drive(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());
  }
}