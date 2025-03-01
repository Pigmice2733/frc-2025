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
  private double xOffset, yOffset, rOffset, calc;

  private PIDController xPID, yPID, rPID;
  private PIDConstants manualConstants, driveConstants, turnConstants;

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
   * @param rOffset    target angle relative from tag
   */
  public DriveVision(Drivetrain drivetrain, Vision vision, double xOffset, double yOffset, double rOffset) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    this.xOffset = xOffset;
    this.yOffset = yOffset;
    this.rOffset = rOffset;

    addRequirements(this.drivetrain, this.vision);
  }

  public DriveVision(Drivetrain drivetrain, Vision vision, double xOffset, double yOffset) {
    this(drivetrain, vision, xOffset, yOffset, 0);
  }

  @Override
  public void initialize() {
    manualConstants = drivetrain.getPidConstants();
    driveConstants = DrivetrainConfig.DRIVE_PID;
    turnConstants = DrivetrainConfig.TURN_PID;

    xPID = new PIDController(driveConstants.kP, driveConstants.kI, driveConstants.kD);
    xPID.setTolerance(DrivetrainConfig.DRIVE_POSITION_TOLERANCE, DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE);

    yPID = new PIDController(driveConstants.kP, driveConstants.kI, driveConstants.kD);
    yPID.setTolerance(DrivetrainConfig.DRIVE_POSITION_TOLERANCE, DrivetrainConfig.DRIVE_VELOCITY_TOLERANCE);

    // rPID = new PIDController(rConstants.kP, rConstants.kI, rConstants.kD);
    rPID = new PIDController(manualConstants.kP, manualConstants.kI, manualConstants.kD);
    rPID.setTolerance(DrivetrainConfig.TURN_POSITION_TOLERANCE, DrivetrainConfig.TURN_VELOCITY_TOLERANCE);

    // System.out.println("starting auto-alignment, P = " + xPID.getP());
    drivetrain.resetPose(new Pose2d());
    getTargetSetpoint();
  }

  @Override
  public void execute() {
    robotPose = drivetrain.getPose();
    calc = rPID.calculate(robotPose.getRotation().getRadians());
    // System.out.println("Position: " + robotPose.getX() + " | calculated value: "
    // + calc);

    if (vision.hasTarget())
      getTargetSetpoint();

    drivetrain.drive(0, 0, calc);
    /*
     * yPID.calculate(robotPose.getY()),
     * -1 * rPID.calculate(robotPose.getRotation().getRadians()));
     */
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
    // System.out.println("Done!");
  }

  @Override
  public boolean isFinished() {
    return rPID.atSetpoint() /* && xPID.atSetpoint() && yPID.atSetpoint() */;
  }

  private void getTargetSetpoint() {
    robotPose = drivetrain.getPose();
    target = vision.getTarget();

    /* The PID controllers use the robot's pose, not the target pose. */
    // xPID.setSetpoint(robotPose.getX() + target.getX() + xOffset);
    // yPID.setSetpoint(robotPose.getY() + target.getY() + yOffset);
    rPID.setSetpoint(robotPose.getRotation().getRadians() + target.getRotation().getRadians() + rOffset);

    Constants.sendNumberToElastic("Drivetrain PID Setpoint", rPID.getSetpoint(), 3);
  }
}
