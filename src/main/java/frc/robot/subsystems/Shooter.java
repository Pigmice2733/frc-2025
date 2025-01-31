package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private SparkMax leftPivot, rightPivot, leftFlywheels, rightFlywheels, indexerMotor;

  public Shooter() {
    leftPivot = new SparkMax(CANConfig.SHOOTER_PIVOT_LEFT, MotorType.kBrushless);
    rightPivot = new SparkMax(CANConfig.SHOOTER_PIVOT_RIGHT, MotorType.kBrushless);
    leftFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_LEFT, MotorType.kBrushless);
    rightFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_RIGHT, MotorType.kBrushless);
    indexerMotor = new SparkMax(CANConfig.INDEXER, MotorType.kBrushless);

    leftPivot.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightPivot.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.SHOOTER_PIVOT_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFlywheels.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFlywheels.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.SHOOTER_FLYWHEELS_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerMotor.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Shooter Left Pivot Speed", leftPivot.get());
    SmartDashboard.putNumber("Shooter Right Pivot Speed", rightPivot.get());
    SmartDashboard.putNumber("Shooter Left Flywheel Speed", leftFlywheels.get());
    SmartDashboard.putNumber("Shooter Right Flywheel Speed", rightFlywheels.get());
    SmartDashboard.putNumber("Shooter Indexer Speed", indexerMotor.get());
  }

  private void setPivot(double speed) {
    leftPivot.set(speed);
  }

  private void setFlywheels(double speed) {
    leftFlywheels.set(speed);
  }

  private void setIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public Command stopMotors() {
    return new InstantCommand(() -> {
      setPivot(0);
      setFlywheels(0);
      setIndexer(0);
    });
  }

  public Command runPivot() {
    return new InstantCommand(() -> setPivot(SystemConfig.SHOOTER_PIVOT_SPEED));
  }

  public Command runFlywheels() {
    return new InstantCommand(() -> setFlywheels(SystemConfig.SHOOTER_FLYWHEEL_SPEED));
  }

  public Command runIndexer() {
    return new InstantCommand(() -> setIndexer(SystemConfig.INDEXER_SPEED));
  }
}
