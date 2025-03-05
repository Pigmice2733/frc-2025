package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private SparkMax pivot, leftFlywheels, rightFlywheels, indexerMotor;
  private PIDController pivotController;
  double targetPivotPosition;
  double indexerSpeed = 0.0;
  // private DigitalInput beamBreak;

  public Shooter() {
    pivot = new SparkMax(CANConfig.SHOOTER_PIVOT, MotorType.kBrushless);
    leftFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_LEFT, MotorType.kBrushless);
    rightFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_RIGHT, MotorType.kBrushless);
    indexerMotor = new SparkMax(CANConfig.INDEXER, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(false);
    pivotConfig.encoder.positionConversionFactor(ShooterConfig.PIVOT_CONVERSION);
    pivot.configure(pivotConfig,
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFlywheels.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFlywheels.configure(new SparkMaxConfig().inverted(true).follow(CANConfig.SHOOTER_FLYWHEELS_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerMotor.configure(new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotController = ShooterConfig.PIVOT_PID;
    pivotController.setTolerance(ShooterConfig.PIVOT_TOLERANCE);
    targetPivotPosition = 0.0;
    indexerSpeed = 0.0;
    // beamBreak = new
    // DigitalInput(Constants.SensorConfig.CORAL_BEAM_BREAK_CHANNEL);
  }

  @Override
  public void periodic() {
    double pivotSpeed = pivotController.calculate(getPivotPosition());
    setPivot(pivotSpeed);
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Shooter Pivot Speed", pivot.get(), 2);
    Constants.sendNumberToElastic("Shooter Pivot Position", pivot.getEncoder().getPosition(), 2);
    Constants.sendNumberToElastic("Shooter Pivot Target", targetPivotPosition, 2);
    Constants.sendNumberToElastic("Shooter Left Flywheel Speed", leftFlywheels.get(), 2);
    Constants.sendNumberToElastic("Shooter Right Flywheel Speed", rightFlywheels.get(), 2);
    Constants.sendNumberToElastic("Shooter Indexer Speed", indexerMotor.get(), 2);

    Constants.sendBooleanToElastic("Has Algae", hasAlgae());
  }

  public void setPivotPositionSetpoint(double targetPivotPosition) {
    this.targetPivotPosition = targetPivotPosition;
    pivotController.setSetpoint(targetPivotPosition);
  }

  public void setPivot(double speed) {

    if ((speed < 0 && getPivotPosition() <= ShooterConfig.PIVOT_LOWER_LIMIT)
        || (speed > 0 && getPivotPosition() >= ShooterConfig.PIVOT_UPPER_LIMIT)) {
      speed = 0;
    }

    pivot.set(speed);
  }

  public void setFlywheels(double speed) {
    leftFlywheels.set(speed);
  }

  public void setIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public double getPivotPosition() {
    return pivot.getEncoder().getPosition();
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

  public boolean flywheelsAtProcessorSpeed() {
    return leftFlywheels.get() >= ShooterConfig.FLYWHEEL_PROCESSOR_SPEED;
  }

  public boolean flywheelsAtNetSpeed() {
    return leftFlywheels.get() >= ShooterConfig.FLYWHEEL_NET_SPEED;
  }

  public boolean hasAlgae() {
    return false;// !beamBreak.get();
  }

  public Command stopMotors() {
    return new InstantCommand(() -> {
      setPivot(0);
      setFlywheels(0);
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
      setPivot(pivot);
      setFlywheels(wheels);
      setIndexer(indexer);
    });
  }

  public Command runIndexerForward() {
    return new InstantCommand(() -> setIndexer(ShooterConfig.INDEXER_SPEED));
  }

  public Command runIndexerReverse() {
    return new InstantCommand(() -> setIndexer(-ShooterConfig.INDEXER_SPEED));
  }
}