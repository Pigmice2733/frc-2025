package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
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
  private SparkMax leftPivot, rightPivot, leftFlywheels, rightFlywheels, indexerMotor;
  private PIDController pivotController;
  private DigitalInput beamBreak;

  public Shooter() {
    leftPivot = new SparkMax(CANConfig.SHOOTER_PIVOT_LEFT, MotorType.kBrushless);
    rightPivot = new SparkMax(CANConfig.SHOOTER_PIVOT_RIGHT, MotorType.kBrushless);
    leftFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_LEFT, MotorType.kBrushless);
    rightFlywheels = new SparkMax(CANConfig.SHOOTER_FLYWHEELS_RIGHT, MotorType.kBrushless);
    indexerMotor = new SparkMax(CANConfig.INDEXER, MotorType.kBrushless);

    leftPivot.configure(new SparkMaxConfig().inverted(false)
        .apply(new AbsoluteEncoderConfig().positionConversionFactor(ShooterConfig.PIVOT_CONVERSION)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightPivot.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.SHOOTER_PIVOT_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    leftFlywheels.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightFlywheels.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.SHOOTER_FLYWHEELS_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerMotor.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotController = ShooterConfig.PIVOT_PID;
    pivotController.setTolerance(ShooterConfig.PIVOT_TOLERANCE);

    beamBreak = new DigitalInput(Constants.SensorConfig.CORAL_BEAM_BREAK_CHANNEL);
  }

  @Override
  public void periodic() {
    if ((leftPivot.get() < 0 && getPivot() <= ShooterConfig.PIVOT_LOWER_LIMIT)
        || (leftPivot.get() > 0 && getPivot() >= ShooterConfig.PIVOT_UPPER_LIMIT)) {
      setPivot(0);
    }

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Shooter Left Pivot Speed", leftPivot.get(), 2);
    Constants.sendNumberToElastic("Shooter Right Pivot Speed", rightPivot.get(), 2);
    Constants.sendNumberToElastic("Shooter Left Flywheel Speed", leftFlywheels.get(), 2);
    Constants.sendNumberToElastic("Shooter Right Flywheel Speed", rightFlywheels.get(), 2);
    Constants.sendNumberToElastic("Shooter Indexer Speed", indexerMotor.get(), 2);

    Constants.sendBooleanToElastic("Has Algae", hasAlgae());
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

  public boolean flywheelsAtProcessorSpeed() {
    return leftFlywheels.get() >= ShooterConfig.FLYWHEEL_PROCESSOR_SPEED;
  }

  public boolean flywheelsAtNetSpeed() {
    return leftFlywheels.get() >= ShooterConfig.FLYWHEEL_NET_SPEED;
  }

  public boolean hasAlgae() {
    return !beamBreak.get();
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