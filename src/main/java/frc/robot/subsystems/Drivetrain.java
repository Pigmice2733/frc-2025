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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
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

  private ShuffleboardTab tab = Constants.ShuffleboardConfig.DRIVETRAIN_TAB;
  private ShuffleboardLayout drivetrainEntries, swerveEntries;
  private GenericEntry robotX, robotY, rotation, frontLeftEntry, frontRightEntry, backLeftEntry, backRightEntry;

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

    drivetrainEntries = tab.getLayout("Drivetrain", BuiltInLayouts.kList).withSize(1, 3)
        .withPosition(0, 0);
    robotX = drivetrainEntries.add("Robot X", 0).withPosition(0, 0).getEntry();
    robotY = drivetrainEntries.add("Robot Y", 0).withPosition(1, 0).getEntry();
    rotation = drivetrainEntries.add("Robot Angle", 0).withPosition(2, 0).getEntry();

    swerveEntries = tab.getLayout("Swerve Modules", BuiltInLayouts.kList).withSize(1, 4)
        .withPosition(1, 0);
    frontLeftEntry = swerveEntries.add("Front Left Encoder Output", 0).withPosition(0, 0).getEntry();
    frontRightEntry = swerveEntries.add("Front Right Encoder Output", 0).withPosition(1, 0).getEntry();
    backLeftEntry = swerveEntries.add("Back Left Encoder Output", 0).withPosition(2, 0).getEntry();
    backRightEntry = swerveEntries.add("Back Right Encoder Output", 0).withPosition(3, 0).getEntry();

    setUpAuto();

    fieldWidget = new Field2d();
    tab.add(fieldWidget);

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
    robotX.setDouble(robotPose.getX());
    robotY.setDouble(robotPose.getY());
    rotation.setValue(robotPose.getRotation().getDegrees());

    frontLeftEntry.setDouble(modules[0].getAbsolutePosition());
    frontRightEntry.setDouble(modules[1].getAbsolutePosition());
    backLeftEntry.setDouble(modules[2].getAbsolutePosition());
    backRightEntry.setDouble(modules[3].getAbsolutePosition());

    fieldWidget.setRobotPose(robotPose);
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
    return new InstantCommand(() -> resetPose(new Pose2d()));
  }

  /**
   * Drives the robot in field-oriented mode by creating a ChassisSpeeds object.
   * Positive X is away from the alliance wall; positive Y is left from the
   * driver's perspective.
   */
  public Command driveCommand(double driveSpeedX, double driveSpeedY, double turnSpeed) {
    return new InstantCommand(() -> drive(driveSpeedX, driveSpeedY, turnSpeed));
  }
}
