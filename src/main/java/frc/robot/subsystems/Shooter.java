package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private SparkMax pivot, leftFlywheel, rightFlywheel, indexerMotor;
  private PIDController pivotController;
  private PIDController flywheelController;
  double targetPivotPosition, targetFlywheelSpeed, pivotSpeed, flywheelSpeed;
  // private DigitalInput beamBreak;

  public Shooter() {
    pivot = new SparkMax(CANConfig.SHOOTER_PIVOT, MotorType.kBrushless);
    leftFlywheel = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_LEFT, MotorType.kBrushless);
    rightFlywheel = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_RIGHT, MotorType.kBrushless);
    indexerMotor = new SparkMax(CANConfig.INDEXER, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(false);
    pivotConfig.encoder.positionConversionFactor(ShooterConfig.PIVOT_CONVERSION);
    pivot.configure(pivotConfig,
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFlywheel.configure(new SparkMaxConfig().inverted(true).idleMode(IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    rightFlywheel.configure(
        new SparkMaxConfig().idleMode(IdleMode.kCoast).follow(CANConfig.SHOOTER_FLYWHEELS_LEFT, true),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerMotor.configure(new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotController = ShooterConfig.PIVOT_PID;
    pivotController.setTolerance(ShooterConfig.PIVOT_TOLERANCE);
    flywheelController = ShooterConfig.FLYWHEEL_PID;
    flywheelController.setTolerance(ShooterConfig.FLYWHEEL_TOLERANCE);
    targetPivotPosition = 0.0;
    targetFlywheelSpeed = 0.0;
    // beamBreak = new
    // DigitalInput(Constants.SensorConfig.CORAL_BEAM_BREAK_CHANNEL);
  }

  @Override
  public void periodic() {
    pivotSpeed = pivotController.calculate(getPivotPosition());
    setPivot(pivotSpeed);
    flywheelSpeed = flywheelController.calculate(leftFlywheel.getEncoder().getVelocity());
    setFlywheels(flywheelSpeed);

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Shooter Pivot Speed", pivot.get(), 2);
    Constants.sendNumberToElastic("Shooter Pivot Position", pivot.getEncoder().getPosition(), 2);
    Constants.sendNumberToElastic("Shooter Pivot Target", targetPivotPosition, 2);
    Constants.sendNumberToElastic("Shooter Left Flywheel Speed", leftFlywheel.get(), 2);
    Constants.sendNumberToElastic("Shooter Right Flywheel Speed", rightFlywheel.get(), 2);
    Constants.sendNumberToElastic("Shooter Flywheel Target Speed", targetFlywheelSpeed, 2);
    Constants.sendNumberToElastic("Shooter Indexer Speed", indexerMotor.get(), 2);

    Constants.sendBooleanToElastic("Has Algae", hasAlgae());
  }

  public void setPivotPositionSetpoint(double targetPos) {
    targetPivotPosition = targetPos;
    pivotController.setSetpoint(targetPos);
  }

  private void setPivot(double speed) {
    if ((speed < 0 && getPivotPosition() <= ShooterConfig.PIVOT_LOWER_LIMIT)
        || (speed > 0 && getPivotPosition() >= ShooterConfig.PIVOT_UPPER_LIMIT)) {
      speed = 0;
    }

    pivot.set(speed);
  }

  public void setTargetFlywheelSpeed(double targetSpeed) {
    targetFlywheelSpeed = targetSpeed;
    flywheelController.setSetpoint(targetFlywheelSpeed);
  }

  private void setFlywheels(double speed) {
    leftFlywheel.set(speed);
  }

  public void setIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public double getPivotPosition() {
    return pivot.getEncoder().getPosition();
  }

  public boolean flywheelsAtSpeed() {
    return flywheelController.atSetpoint();
  }

  public boolean hasAlgae() {
    return false;// !beamBreak.get();
  }

  public Command stopMotors() {
    return new InstantCommand(() -> {
      setPivotPositionSetpoint(0);
      setTargetFlywheelSpeed(0);
      setIndexer(0);
    });
  }

  /**
   * 
   * @param pivot   speed for pivot motors
   * @param wheels  speed for flywheel motors
   * @param indexer speed for indexer motor
   * @return command to set speeds
   */
  public Command manualSpeed(double pivot, double wheels, double indexer) {
    return new InstantCommand(() -> {
      setPivotPositionSetpoint(pivot);
      setTargetFlywheelSpeed(wheels);
      setIndexer(indexer);
    });
  }

  public Command runIndexerOut() {
    return Commands.runOnce(() -> setIndexer(ShooterConfig.INDEXER_SPEED), this);
  }

  public Command runIndexerIn() {
    return Commands.runOnce(() -> setIndexer(-ShooterConfig.INDEXER_SPEED), this);
  }

  public Command stopIndexer() {
    return Commands.runOnce(() -> setIndexer(0), this);
  }

  public Command changePivotSetpoint(DoubleSupplier delta) {
    return Commands.run(() -> setPivotPositionSetpoint(pivotController.getSetpoint() + delta.getAsDouble()), this);
  }
}