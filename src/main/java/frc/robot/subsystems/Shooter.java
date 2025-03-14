package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
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
  private double targetPivotPosition, targetFlywheelSpeed, pivotSpeed;
  private DigitalInput upperAlgaeLimitSwitch, lowerAlgaeLimitSwitch;

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
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig.inverted(true).idleMode(IdleMode.kCoast);
    flywheelConfig.closedLoop.p(0.003).d(0.001).velocityFF(0.0006);
    leftFlywheel.configure(flywheelConfig,
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFlywheel.configure(
        new SparkMaxConfig().idleMode(IdleMode.kCoast).follow(CANConfig.SHOOTER_FLYWHEELS_LEFT, true),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerMotor.configure(new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotController = ShooterConfig.PIVOT_PID;
    pivotController.setTolerance(ShooterConfig.PIVOT_TOLERANCE);
    targetPivotPosition = 0.0;
    targetFlywheelSpeed = 0.0;
    upperAlgaeLimitSwitch = new DigitalInput(Constants.SensorConfig.SHOOTER_UPPER_ALGAE_LIMIT_CHANNEL);
    lowerAlgaeLimitSwitch = new DigitalInput(Constants.SensorConfig.SHOOTER_LOWER_ALGAE_LIMIT_CHANNEL);

    // Constants.sendNumberToElastic("Shooter Flywheel Speed", 0, 1);
    // Constants.sendNumberToElastic("Flywheels P", 0, 0);
    // Constants.sendNumberToElastic("Flywheels I", 0, 0);
    // Constants.sendNumberToElastic("Flywheels D", 0, 0);
  }

  @Override
  public void periodic() {
    setPivot(calculatePivot());

    if (targetFlywheelSpeed == 0) {
      leftFlywheel.getClosedLoopController().setReference(0, ControlType.kVelocity);
    }

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Shooter Pivot Speed", pivot.get(), 2);
    Constants.sendNumberToElastic("Shooter Pivot Position", pivot.getEncoder().getPosition(), 2);
    Constants.sendNumberToElastic("Shooter Pivot Target", targetPivotPosition, 2);
    Constants.sendNumberToElastic("Shooter Left Flywheel Speed", leftFlywheel.getEncoder().getVelocity(), 2);
    Constants.sendNumberToElastic("Shooter Right Flywheel Speed", rightFlywheel.getEncoder().getVelocity(), 2);
    Constants.sendNumberToElastic("Shooter Flywheel Target Speed", targetFlywheelSpeed, 2);
    Constants.sendNumberToElastic("Shooter Indexer Speed", indexerMotor.get(), 2);

    Constants.sendBooleanToElastic("Has Algae", hasAlgae());
    Constants.sendBooleanToElastic("Shooter Upper Limit Switch", isUpperAlgaeLimitSwitchPressed());
    Constants.sendBooleanToElastic("Shooter Lower Limit Switch", isLowerAlgaeLimitSwitchPressed());
    Constants.sendBooleanToElastic("Shooter Flywheels At Speed", flywheelsAtSpeed());

    // setTargetFlywheelSpeed(SmartDashboard.getNumber("Shooter Flywheel Speed",
    // 0));
    // flywheelController.setP(SmartDashboard.getNumber("Flywheels P", 0));
    // flywheelController.setI(SmartDashboard.getNumber("Flywheels I", 0));
    // flywheelController.setD(SmartDashboard.getNumber("Flywheels D", 0));
  }

  public void setPivotPositionSetpoint(double targetPos) {
    targetPivotPosition = targetPos;

    // prevent setpoint from being out of range
    if (targetPivotPosition < ShooterConfig.PIVOT_LOWER_LIMIT) {
      pivotController.setSetpoint(ShooterConfig.PIVOT_LOWER_LIMIT);
    } else if (targetPivotPosition > ShooterConfig.PIVOT_UPPER_LIMIT) {
      pivotController.setSetpoint(ShooterConfig.PIVOT_UPPER_LIMIT);
    } else {
      pivotController.setSetpoint(targetPivotPosition);
    }
  }

  public void changeSetpoint(double delta) {
    if (delta != 0) {
      setPivotPositionSetpoint(getPivotPosition() + delta);
    }
  }

  public boolean pivotAtSetpoint() {
    return pivotController.atSetpoint();
  }

  /** Returns the calculated output based on the current angle and velocity. */
  public double calculatePivot() {
    pivotSpeed = pivotController.calculate(getPivotPosition());

    if (getPivotPosition() <= ShooterConfig.PIVOT_LOWER_LIMIT && pivotSpeed < 0) {
      return 0;
    }
    if (getPivotPosition() >= ShooterConfig.PIVOT_UPPER_LIMIT && pivotSpeed > 0) {
      return 0;
    }

    return pivotSpeed;
  }

  private void setPivot(double speed) {
    pivotSpeed = speed;
    pivot.set(speed);
  }

  public void setTargetFlywheelSpeed(double targetSpeed) {
    targetFlywheelSpeed = targetSpeed;
    leftFlywheel.getClosedLoopController().setReference(targetFlywheelSpeed, ControlType.kVelocity);
  }

  public void setIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public double getPivotPosition() {
    return pivot.getEncoder().getPosition();
  }

  public boolean flywheelsAtSpeed() {
    if (targetFlywheelSpeed > 10) {
      return leftFlywheel.getEncoder().getVelocity() > (targetFlywheelSpeed - ShooterConfig.FLYWHEEL_TOLERANCE);
    } else if (targetFlywheelSpeed < -10) {
      return leftFlywheel.getEncoder().getVelocity() < (targetFlywheelSpeed + ShooterConfig.FLYWHEEL_TOLERANCE);
    }
    return false;
  }

  public boolean hasAlgae() {
    return isUpperAlgaeLimitSwitchPressed() || isLowerAlgaeLimitSwitchPressed();
  }

  public boolean isAlgaeCentered() {
    return isUpperAlgaeLimitSwitchPressed() && isLowerAlgaeLimitSwitchPressed();
  }

  public boolean isUpperAlgaeLimitSwitchPressed() {
    return !upperAlgaeLimitSwitch.get();
  }

  public boolean isLowerAlgaeLimitSwitchPressed() {
    return !lowerAlgaeLimitSwitch.get();
  }

  public Command stopMotors() {
    return new InstantCommand(() -> {
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