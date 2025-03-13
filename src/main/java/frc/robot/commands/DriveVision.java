package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveVision extends Command {
  private Drivetrain drivetrain;
  private Vision vision;
  private double xOffset, yOffset;

  private PIDController xPID, yPID;
  private PIDConstants pidConstants;
  private int id, counter;

  private Pose2d target, robotPose;

  /**
   * A command that drives the robot to the given position with respect to the
   * target. Tolerances have been set generously high to avoid spasmodic behavior,
   * but such a paradigm intended for precision will need lower tolerances.
   * 
   * @param drivetrain drivetrain subsystem
   * @param vision     vision subsystem
   * @param xOffset    target distance from tag in the X direction
   * @param yOffset    target distance from tag in the Y direction
   */
  public DriveVision(Drivetrain drivetrain, Vision vision, double xOffset, double yOffset) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    this.xOffset = xOffset;
    this.yOffset = yOffset;

    counter = 0;

    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    pidConstants = DrivetrainConfig.DRIVE_PID;

    xPID = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    xPID.setTolerance(DrivetrainConfig.DRIVE_POSITION_TOLERANCE, DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE);

    yPID = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    yPID.setTolerance(DrivetrainConfig.DRIVE_POSITION_TOLERANCE, DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE);

    drivetrain.resetPose(new Pose2d());
    id = vision.getTargetID();
    getTargetSetpoint();

    System.out.println("drive started");
  }

  @Override
  public void execute() {
    counter++;

    robotPose = drivetrain.getPose();

    if (vision.hasTarget() && vision.getTargetID() == id && counter >= 5) {
      // getTargetSetpoint();
      counter = 0;
    }

    drivetrain.drive(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY()), 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
    drivetrain.getSwerve().lockPose();
    System.out.println("drive finished");
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint();
  }

  private void getTargetSetpoint() {
    robotPose = drivetrain.getPose();
    target = vision.getTargetPose();

    /* The PID controllers use the robot's pose, not the target pose. */
    xPID.setSetpoint(robotPose.getX() + target.getX() + xOffset);
    yPID.setSetpoint(robotPose.getY() - target.getY() + yOffset);

    Constants.sendNumberToElastic("Drivetrain X Setpoint", xPID.getSetpoint(), 3);
    Constants.sendNumberToElastic("Drivetrain Y Setpoint", yPID.getSetpoint(), 3);
  }
}
