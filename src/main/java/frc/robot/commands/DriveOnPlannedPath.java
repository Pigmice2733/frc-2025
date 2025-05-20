package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class DriveOnPlannedPath extends Command {
  PathPlannerTrajectory trajectory;
  Drivetrain d;
  PIDConstants drivePID, turnPID;
  PIDController xPID, yPID, rPID;
  Pose2d currentPose;

  public DriveOnPlannedPath(Drivetrain d, String path) {
    this.d = d;
    try {
      trajectory = PathPlannerPath.fromPathFile(path).generateTrajectory(null, null, null);
    } catch (Exception e) {
      e.printStackTrace();
    }

    drivePID = DrivetrainConfig.DRIVE_PID;
    turnPID = DrivetrainConfig.TURN_PID;

    xPID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    xPID.setTolerance(0.1);

    yPID = new PIDController(drivePID.kP, drivePID.kI, drivePID.kD);
    yPID.setTolerance(0.1);

    rPID = new PIDController(turnPID.kP, turnPID.kI, turnPID.kD);
    rPID.setTolerance(0.5);

    addRequirements(d);
  }

  @Override
  public void execute() {
    Pose2d pos = trajectory.sample(Timer.getMatchTime()).pose;
    currentPose = d.getPose();

    xPID.setSetpoint(pos.getX());
    yPID.setSetpoint(pos.getY());
    rPID.setSetpoint(pos.getRotation().getRadians());

    d.driveField(
        xPID.calculate(currentPose.getX()),
        yPID.calculate(currentPose.getY()),
        rPID.calculate(currentPose.getRotation().getRadians()));
  }

  @Override
  public boolean isFinished() {
    return d.getPose() == trajectory.getEndState().pose;
  }
}
