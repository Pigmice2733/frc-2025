package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
import frc.robot.Constants.*;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;

public class TemplateSubsystem extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(MotorType.kBrushless, 0);

  private ShuffleboardTab tab = ShuffleboardConfig.DRIVETRAIN_TAB;
  private ShuffleboardLayout entryList;
  private GenericEntry entry;

  public TemplateSubsystem() {
    entryList = tab.getLayout("Subsystem", BuiltInLayouts.kList).withSize(1, 4)
        .withPosition(1, 0);
    entry = entryList.add("Name", 0).withPosition(0, 0).getEntry();
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    entry.setDouble(0);
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> motor.set(0));
  }

  public Command startMotor() {
    return new InstantCommand(() -> motor.set(1));
  }
}
