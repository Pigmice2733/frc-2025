package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase {
  private SwerveDrive swerve;
  private final SwerveIMU gyro;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private Pose2d robotPose;
  private final Field2d fieldWidget;

  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModule[] modules = new SwerveModule[4];

  public Drivetrain() {
    try {
      swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(DrivetrainConfig.MAX_DRIVE_SPEED);
    } catch (IOException e) {
      e.printStackTrace();
    }

    modules = swerve.getModules();
    modulePositions = swerve.getModulePositions();
    kinematics = swerve.kinematics;
    gyro = swerve.getGyro();
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation3d().toRotation2d(), modulePositions);
    robotPose = new Pose2d();
    resetPose(robotPose);

    setUpAuto();

    fieldWidget = new Field2d();
    PathPlannerLogging.setLogActivePathCallback((pose) -> fieldWidget.getObject("target pose").setPoses(pose));
  }

  @Override
  public void periodic() {
    robotPose = getPose();

    updateEntries();

    updateModulePositions();
    odometry.update(gyro.getRotation3d().toRotation2d(), modulePositions);
  }

  /**
   * Sets the configuration of the drivetrain for the AutoBuilder.
   */
  public void setUpAuto() {
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(() -> {
      Pose2d pose = getPose();
      return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(-pose.getRotation().getRadians()));
    },
        (pose) -> swerve.resetOdometry(pose),
        swerve::getRobotVelocity,
        (speed) -> drive(speed.vxMetersPerSecond, speed.vyMetersPerSecond, -speed.omegaRadiansPerSecond),
        new PPHolonomicDriveController(DrivetrainConfig.DRIVE_PID, DrivetrainConfig.TURN_PID),
        config,
        () -> (DriverStation.getAlliance().get() == Alliance.Red),
        this);

  }

  /**
   * Updates the stored values for module positions to the current positions of
   * the modules.
   */
  private void updateModulePositions() {
    modulePositions[0] = modules[0].getPosition();
    modulePositions[1] = modules[1].getPosition();
    modulePositions[2] = modules[2].getPosition();
    modulePositions[3] = modules[3].getPosition();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Robot X", robotPose.getX());
    SmartDashboard.putNumber("Robot Y", robotPose.getY());
    SmartDashboard.putNumber("Robot Angle", robotPose.getRotation().getDegrees());

    SmartDashboard.putNumber("Front Left Encoder Output", modules[0].getAbsolutePosition());
    SmartDashboard.putNumber("Front Right Encoder Output", modules[1].getAbsolutePosition());
    SmartDashboard.putNumber("Back Left Encoder Output", modules[2].getAbsolutePosition());
    SmartDashboard.putNumber("Back Right Encoder Output", modules[3].getAbsolutePosition());

    fieldWidget.setRobotPose(robotPose);
    SmartDashboard.putData("Field", fieldWidget);
  }

  /** Returns the drivetrain as a SwerveDrive object. */
  public SwerveDrive getSwerve() {
    return swerve;
  }

  /** Returns the current pose of the robot. */
  public Pose2d getPose() {
    return new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), swerve.getPose().getRotation());
  }

  /** Sets the robot odometry to the given pose. */
  public void resetPose(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  /**
   * Drives the robot in field-oriented mode by creating a ChassisSpeeds object.
   * Positive X is away from the alliance wall; positive Y is left from the
   * driver's perspective.
   */
  public void drive(double driveSpeedX, double driveSpeedY, double turnSpeed) {
    // System.out.println("Driving. x speed " + driveSpeedX + ", y speed " +
    // driveSpeedY + ", turn speed " + turnSpeed);
    swerve.driveFieldOriented(new ChassisSpeeds(driveSpeedX, driveSpeedY, turnSpeed));
  }

  public Command reset() {
    return new InstantCommand(() -> resetPose(new Pose2d()), this);
  }

  /**
   * Drives the robot in field-oriented mode by creating a ChassisSpeeds object.
   * Positive X is away from the alliance wall; positive Y is left from the
   * driver's perspective.
   */
  public Command driveCommand(double driveSpeedX, double driveSpeedY, double turnSpeed) {
    return new InstantCommand(() -> drive(driveSpeedX, driveSpeedY, turnSpeed), this);
  }

}
