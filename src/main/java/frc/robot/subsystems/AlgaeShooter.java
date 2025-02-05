package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class AlgaeShooter extends SubsystemBase {
  private SparkMax leftPivot, rightPivot, leftFlywheels, rightFlywheels, indexerMotor;
  private PIDController pivotController;

  public AlgaeShooter() {
    leftPivot = new SparkMax(CANConfig.SHOOTER_PIVOT_LEFT, MotorType.kBrushless);
    rightPivot = new SparkMax(CANConfig.SHOOTER_PIVOT_RIGHT, MotorType.kBrushless);
    leftFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_LEFT, MotorType.kBrushless);
    rightFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_RIGHT, MotorType.kBrushless);
    indexerMotor = new SparkMax(CANConfig.INDEXER, MotorType.kBrushless);

    leftPivot.configure(new SparkMaxConfig().inverted(false)
        .apply(new AbsoluteEncoderConfig().positionConversionFactor(SystemConfig.SHOOTER_CONVERSION)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightPivot.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.SHOOTER_PIVOT_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFlywheels.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFlywheels.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.SHOOTER_FLYWHEELS_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerMotor.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotController = SystemConfig.SHOOTER_PID;
    pivotController.setTolerance(SystemConfig.SHOOTER_TOLERANCE);

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

  public void setPivot(double speed) {
    leftPivot.set(speed);
  }

  public void setFlywheels(double speed) {
    leftFlywheels.set(speed);
  }

  public void setIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public double getPivot() {
    return leftPivot.getEncoder().getPosition();
  }

  public double getFlywheels() {
    return leftFlywheels.getEncoder().getPosition();
  }

  public double getIndexer() {
    return indexerMotor.getEncoder().getPosition();
  }

  public PIDController getController() {
    return pivotController;
  }

  public Command stopMotors() {
    return new InstantCommand(() -> {
      setPivot(0);
      setFlywheels(0);
      setIndexer(0);
    });
  }

  public Command manualSpeed(double speed) {
    return new InstantCommand(() -> {
      setPivot(speed);
      setFlywheels(speed);
      setIndexer(speed);
    });
  }

  public Command runIndexerForward() {
    return new InstantCommand(() -> setIndexer(SystemConfig.INDEXER_SPEED));
  }

  public Command runIndexerReverse() {
    return new InstantCommand(() -> setIndexer(-SystemConfig.INDEXER_SPEED));
  }
}