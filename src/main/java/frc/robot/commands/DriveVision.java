package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveVision extends Command {
  private Drivetrain drivetrain;
  private Vision vision;
  private double xOffset, yOffset, kP;
  private double xSetpoint, xError, ySetpoint, yError;

  // private PIDController xPID, yPID, testPID;
  // private PIDConstants pidConstants;
  private int id, counter;

  private Pose2d target, robotPose;
  private ChassisSpeeds speed;

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

    xSetpoint = ySetpoint = xError = yError = 0;
    counter = 0;
    speed = new ChassisSpeeds();

    Constants.sendNumberToElastic("Drive kP", DrivetrainConfig.DRIVE_P, 2);

    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    // pidConstants = DrivetrainConfig.DRIVE_PID;

    // testPID = (PIDController) SmartDashboard.getData("Drivetrain Drive PID");
    // xPID = new PIDController(testPID.getP(), testPID.getI(), testPID.getD());
    // xPID = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    // xPID.setTolerance(DrivetrainConfig.DRIVE_POSITION_TOLERANCE,
    // DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE);

    // yPID = new PIDController(testPID.getP(), testPID.getI(), testPID.getD());
    // yPID = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    // yPID.setTolerance(DrivetrainConfig.DRIVE_POSITION_TOLERANCE,
    // DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE);

    kP = SmartDashboard.getNumber("Drive kP", 0);

    drivetrain.resetPose(new Pose2d());
    id = vision.getTargetID();
    getTargetSetpoint();

    System.out.println("drive started");
  }

  @Override
  public void execute() {
    counter++;

    robotPose = drivetrain.getPose();
    speed = drivetrain.getSwerve().getFieldVelocity();
    System.out.println(speed.toString());

    if (vision.hasTarget() && vision.getTargetID() == id && counter >= 5) {
      getTargetSetpoint();
      counter = 0;
    }

    xError = xSetpoint - robotPose.getX();
    yError = ySetpoint - robotPose.getY();

    drivetrain.driveField(xError * kP, yError * kP, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveField(0, 0, 0);
    drivetrain.getSwerve().lockPose();
    System.out.println("drive finished");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(speed.vxMetersPerSecond) < DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE &&
        Math.abs(speed.vyMetersPerSecond) < DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE
        && ((Math.abs(xError) < DrivetrainConfig.DRIVE_POSITION_TOLERANCE
            && Math.abs(yError) < DrivetrainConfig.DRIVE_POSITION_TOLERANCE)
            || !vision.hasTarget());
  }

  private void getTargetSetpoint() {
    drivetrain.resetPose(new Pose2d());
    target = vision.getTargetPose();

    xSetpoint = target.getX() + xOffset;
    ySetpoint = -1 * target.getY() + yOffset;

    // System.out.println("target offset: " + target.getX() + " setpoint: "
    // + xPID.getSetpoint() + " error: " + (xPID.getSetpoint() - target.getX()));

    Constants.sendNumberToElastic("Drivetrain X Setpoint", xSetpoint, 3);
    Constants.sendNumberToElastic("Drivetrain Y Setpoint", ySetpoint, 3);
  }
}
