package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveAndTurnFieldRelative extends Command {
  private Drivetrain drivetrain;
  private Vision vision;
  private CommandXboxController controller;

  private double xError, yError, rError,
      driveP, turnP;
  private int id, counter;

  private Pose2d target;
  private ChassisSpeeds speed;
  private Transform2d offsetTransform;

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
  public DriveAndTurnFieldRelative(Drivetrain drivetrain, Vision vision, CommandXboxController controller,
      double xOffset,
      double yOffset, double rOffset) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.controller = controller;
    offsetTransform = new Transform2d(new Translation2d(xOffset, yOffset),
        new Rotation2d(Units.degreesToRadians(rOffset)));

    xError = yError = rError = 0;
    counter = 0;
    speed = new ChassisSpeeds();

    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    if (!vision.hasTarget())
      controller.setRumble(RumbleType.kBothRumble, 0.5);

    driveP = DrivetrainConfig.DRIVE_P;
    turnP = DrivetrainConfig.TURN_P;
    id = vision.getTargetID();
    getTargetSetpoint();

    System.out.println("DriveAndTurnFieldRelative started");
  }

  @Override
  public void execute() {
    counter++;

    Pose2d robotPose = drivetrain.getPose();
    if (vision.hasTarget() && vision.getTargetID() == id && counter >= 5) {
      getTargetSetpoint();
      counter = 0;
    }

    xError = target.getX() - robotPose.getX();
    yError = target.getY() - robotPose.getY();
    rError = target.getRotation().getDegrees() - robotPose.getRotation().getDegrees();

    Constants.sendNumberToElastic("Drivetrain X Setpoint", target.getX(), 3);
    Constants.sendNumberToElastic("Drivetrain Y Setpoint", target.getY(), 3);
    Constants.sendNumberToElastic("Drivetrain Turn Setpoint", target.getRotation().getDegrees(), 3);
    Constants.sendNumberToElastic("Drivetrain X Error", xError, 3);
    Constants.sendNumberToElastic("Drivetrain Y Error", yError, 3);
    Constants.sendNumberToElastic("Drivetrain Turn Error", rError, 3);

    drivetrain.driveField(xError * driveP, yError * driveP, Units.degreesToRadians(rError) * turnP);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveField(0, 0, 0);
    drivetrain.getSwerve().lockPose();
    controller.setRumble(RumbleType.kBothRumble, 0);
    System.out.println("DriveAndTurnFieldRelative finished");
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

  private Pose2d getTargetPose(Pose2d robotPose, Pose2d visionTarget, Transform2d offsetTransform) {
    Pose2d robotPoseAtTag = robotPose.plus(new Transform2d(visionTarget.getTranslation(), visionTarget.getRotation()));
    return robotPoseAtTag.plus(offsetTransform);
  }

  private void getTargetSetpoint() {
    Pose2d visionPose = vision.getTargetPose();
    Pose2d translatedVisionPose = new Pose2d(new Translation2d(visionPose.getX(), -1 * visionPose.getY()),
        visionPose.getRotation());
    target = getTargetPose(drivetrain.getPose(), translatedVisionPose, offsetTransform);
  }
}
