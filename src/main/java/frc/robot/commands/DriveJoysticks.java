package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class DriveJoysticks extends Command {
  private Drivetrain dvt;
  private DoubleSupplier x, y, r;
  private BooleanSupplier robotOriented;

  public DriveJoysticks(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier turnSpeed,
      BooleanSupplier robotOriented) {
    dvt = drivetrain;
    x = xSpeed;
    y = ySpeed;
    r = turnSpeed;
    this.robotOriented = robotOriented;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (robotOriented.getAsBoolean()) {
      dvt.driveRobot(x.getAsDouble() * DrivetrainConfig.SLOWMODE_FACTOR, y.getAsDouble()
          * DrivetrainConfig.SLOWMODE_FACTOR, r.getAsDouble() * DrivetrainConfig.SLOWMODE_FACTOR);
    } else {
      dvt.driveField(-x.getAsDouble(), -y.getAsDouble(), r.getAsDouble());
    }
  }
}