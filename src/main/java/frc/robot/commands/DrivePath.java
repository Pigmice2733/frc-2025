package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class DrivePath extends Command {
  private final Drivetrain drivetrain;
  private Pose2d endPose, currentPose;
  private Transform2d path;

  private PIDController xPID, yPID, rPID;
  private PIDConstants drivePID, turnPID;

  /**
   * Drives to the given location w.r.t. the robot's current position.
   * 
   * @param drivetrain the drivetrain subsystem
   * @param path       position to drive to, in the robot's reference frame
   */
  public DrivePath(Drivetrain drivetrain, Transform2d path) {
    this.drivetrain = drivetrain;
    this.path = path;

    drivePID = DrivetrainConfig.DRIVE_PID;
    turnPID = DrivetrainConfig.TURN_PID;

    xPID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    xPID.setTolerance(0.1);

    yPID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    yPID.setTolerance(0.1);

    rPID = new PIDController(turnPID.kP, turnPID.kI, turnPID.kD);
    rPID.setTolerance(0.5);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    currentPose = drivetrain.getPose();
    endPose = currentPose.transformBy(path);

    xPID.setSetpoint(endPose.getX());
    yPID.setSetpoint(endPose.getY());
    rPID.setSetpoint(endPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    currentPose = drivetrain.getPose();
    drivetrain.drive(
        xPID.calculate(currentPose.getX()),
        yPID.calculate(currentPose.getY()),
        rPID.calculate(currentPose.getRotation().getRadians()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && rPID.atSetpoint();
  }
}