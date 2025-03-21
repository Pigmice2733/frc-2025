package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveAndTurn extends Command {
  private Drivetrain drivetrain;
  private Vision vision;
  private CommandXboxController controller;

  private double xOffset, yOffset, rOffset,
      xSetpoint, ySetpoint, rSetpoint,
      xError, yError, rError,
      driveP, turnP;
  private double initialTargetAngle;
  private Pose2d initialRobotPose;
  // private PIDController xPID, yPID, testPID;
  // private PIDConstants pidConstants;
  private int id, counter;

  private Pose2d target, robotPose;
  private ChassisSpeeds speed;

  /**
   * A command that drives the robot to the given position and rotation with
   * respect to the target using simple error correction. If no target is visible,
   * the given controller rumbles to alert the driver.
   * 
   * @param drivetrain drivetrain subsystem
   * @param vision     vision subsystem
   * @param controller driver XBox controller
   * @param xOffset    target distance from tag in the X direction (m)
   * @param yOffset    target distance from tag in the Y direction (m)
   * @param rOffset    target rotation from tag (deg)
   */
  public DriveAndTurn(Drivetrain drivetrain, Vision vision, CommandXboxController controller, double xOffset,
      double yOffset, double rOffset) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.controller = controller;

    this.xOffset = xOffset;
    this.yOffset = yOffset;
    this.rOffset = rOffset;

    xSetpoint = ySetpoint = rSetpoint = xError = yError = rError = 0;
    counter = 0;
    speed = new ChassisSpeeds();

    initialTargetAngle = vision.getTargetPose().getRotation().getDegrees();
    initialRobotPose = drivetrain.getPose();
    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    if (!vision.hasTarget())
      controller.setRumble(RumbleType.kBothRumble, 0.5);

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

    driveP = DrivetrainConfig.DRIVE_P;
    turnP = DrivetrainConfig.TURN_P;

    drivetrain.resetPose(new Pose2d());
    id = vision.getTargetID();
    getTargetSetpoint();

    System.out.println("joint started");
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
    rError = rSetpoint - robotPose.getRotation().getDegrees();
    Constants.sendNumberToElastic("Drive And Turn Initial Angle", initialTargetAngle, 3);
    Constants.sendNumberToElastic("Drive And Turn rError", rError, 3);
    Constants.sendNumberToElastic("Drive And Turn angleDelta", getAngleDelta(), 3);

    drivetrain.driveField(xError * driveP, yError * driveP, Units.degreesToRadians(rError) * turnP);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveField(0, 0, 0);
    drivetrain.getSwerve().lockPose();
    controller.setRumble(RumbleType.kBothRumble, 0);
    Pose2d rotatedPose = initialRobotPose.rotateBy(new Rotation2d(initialTargetAngle));
    drivetrain.resetPose(rotatedPose);
    System.out.println("joint finished");
  }

  public double getInitialTargetAngle() {
    return initialTargetAngle;
  }

  public double getAngleDelta() {
    return initialTargetAngle - rError;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(speed.vxMetersPerSecond) < DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE &&
        Math.abs(speed.vyMetersPerSecond) < DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE &&
        Math.abs(speed.omegaRadiansPerSecond) < DrivetrainConfig.TURN_VELOCITY_TOLERANCE &&
        ((Math.abs(xError) < DrivetrainConfig.DRIVE_POSITION_TOLERANCE
            && Math.abs(yError) < DrivetrainConfig.DRIVE_POSITION_TOLERANCE
            && Math.abs(rError) < DrivetrainConfig.TURN_POSITION_TOLERANCE)
            || !vision.hasTarget());
  }

  private void getTargetSetpoint() {
    drivetrain.resetPose(new Pose2d());
    target = vision.getTargetPose();

    xSetpoint = target.getX() + xOffset;
    ySetpoint = -1 * target.getY() + yOffset;
    rSetpoint = target.getRotation().getDegrees() + rOffset;

    // System.out.println("target offset: " + target.getX() + " setpoint: "
    // + xPID.getSetpoint() + " error: " + (xPID.getSetpoint() - target.getX()));

    Constants.sendNumberToElastic("Drivetrain X Setpoint", xSetpoint, 3);
    Constants.sendNumberToElastic("Drivetrain Y Setpoint", ySetpoint, 3);
    Constants.sendNumberToElastic("Drivetrain Turn Setpoint", rSetpoint, 3);
  }
}
