package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {
  private SwerveDrive swerve;
  private Pose2d robotPose;
  private final Field2d fieldWidget;
  private PIDConstants pidConstants;

  public Drivetrain() {
    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
        Meter.of(4)),
        Rotation2d.fromDegrees(0))
        : new Pose2d(new Translation2d(Meter.of(16),
            Meter.of(4)),
            Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(DrivetrainConfig.MAX_DRIVE_SPEED, startingPose);
    } catch (IOException e) {
      e.printStackTrace();
    }
    swerve.setHeadingCorrection(false);
    swerve.setCosineCompensator(false);
    swerve.setAngularVelocityCompensation(true, true, 0.1);
    swerve.setModuleEncoderAutoSynchronize(false, 1);
    swerve.getGyro().factoryDefault();
    // gyro.setInverted(true);
    setUpAuto();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));

    fieldWidget = new Field2d();
    PathPlannerLogging.setLogActivePathCallback((pose) -> fieldWidget.getObject("target pose").setPoses(pose));

    Constants.sendNumberToElastic("Drivetrain P", 0, 3);
    Constants.sendNumberToElastic("Drivetrain I", 0, 3);
    Constants.sendNumberToElastic("Drivetrain D", 0, 3);

  }

  @Override
  public void periodic() {
    updateVision();

    robotPose = getPose();
    LimelightHelpers.SetRobotOrientation("", robotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    updateEntries();
  }

  private void updateVision() {
    if (!LimelightHelpers.getTV("")) {
      return;
    }
    PoseEstimate botPos = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    swerve.addVisionMeasurement(botPos.pose, botPos.timestampSeconds);
  }

  /**
   * Sets the configuration of the drivetrain for the AutoBuilder.
   */
  public void setUpAuto() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          swerve::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerve::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerve.drive(
                  speedsRobotRelative,
                  swerve.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerve.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          this::isRedAlliance,
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Robot X", robotPose.getX(), 3);
    Constants.sendNumberToElastic("Robot Y", robotPose.getY(), 3);
    Constants.sendNumberToElastic("Robot Angle", robotPose.getRotation().getDegrees(), 1);

    Constants.sendNumberToElastic("Drivetrain Linear Speed",
        Math.hypot(swerve.getFieldVelocity().vxMetersPerSecond, swerve.getFieldVelocity().vyMetersPerSecond), 3);
    Constants.sendNumberToElastic("Drivetrain Angular Speed", swerve.getFieldVelocity().omegaRadiansPerSecond, 3);

    pidConstants = new PIDConstants(SmartDashboard.getNumber("Drivetrain P", 0),
        SmartDashboard.getNumber("Drivetrain I", 0), SmartDashboard.getNumber("Drivetrain D", 0));

    fieldWidget.setRobotPose(robotPose);
    SmartDashboard.putData("Field", fieldWidget);
  }

  /** Returns the drivetrain as a SwerveDrive object. */
  public SwerveDrive getSwerve() {
    return swerve;
  }

  /** Returns the current pose of the robot. */
  public Pose2d getPose() {
    return swerve.getPose();
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
  public void driveField(double driveSpeedX, double driveSpeedY, double turnSpeed) {
    // System.out.println("Driving. x speed " + driveSpeedX + ", y speed " +
    // driveSpeedY + ", turn speed " + turnSpeed);
    swerve.driveFieldOriented(new ChassisSpeeds(driveSpeedX, driveSpeedY, turnSpeed));
  }

  /**
   * Drives the robot in robot-oriented mode by creating a ChassisSpeeds object.
   * Positive X is robot's forward; positive Y is robot's left.
   */
  public void driveRobot(double driveSpeedX, double driveSpeedY, double turnSpeed) {
    swerve.drive(new ChassisSpeeds(driveSpeedX, driveSpeedY, turnSpeed));
  }

  public Command reset() {
    return new InstantCommand(() -> resetPose(new Pose2d()), this);
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(swerve.getMaximumChassisVelocity(), 4.0,
        swerve.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0)); // Goal end velocity in meters/sec
  }

  /**
   * Drives the robot in field-oriented mode by creating a ChassisSpeeds object.
   * Positive X is away from the alliance wall; positive Y is left from the
   * driver's perspective.
   */
  public Command driveCommand(double driveSpeedX, double driveSpeedY, double turnSpeed) {
    return Commands.run(() -> driveField(driveSpeedX, driveSpeedY, turnSpeed), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> driveField(0, 0, 0), this);
  }

  public PIDConstants getPidConstants() {
    return pidConstants;
  }

  public void setPidConstants(PIDConstants pidConstants) {
    this.pidConstants = pidConstants;
  }

  public Command simpleAuto() {
    return Commands.sequence(
        driveCommand(DrivetrainConfig.MAX_DRIVE_SPEED, 0, 0),
        Commands.waitUntil(() -> (robotPose.getX() > 2)),
        stop());
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      swerve.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }
}
