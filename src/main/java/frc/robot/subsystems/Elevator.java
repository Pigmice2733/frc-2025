package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private DigitalInput limitSwitch;
  private PIDController heightController;

  public Elevator() {
    leftMotor = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
    rightMotor = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);

    leftMotor.configure(new SparkMaxConfig().inverted(false)
        .apply(new AbsoluteEncoderConfig().positionConversionFactor(SystemConfig.ELEVATOR_CONVERSION)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.ELEVATOR_LEFT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    heightController = SystemConfig.ELEVATOR_PID;
    heightController.setTolerance(SystemConfig.ELEVATOR_TOLERANCE);

    limitSwitch = new DigitalInput(CANConfig.LIMIT_SWITCH_CHANNEL);
  }

  @Override
  public void periodic() {
    if (leftMotor.get() < 0 && getSwitch()) {
      setSpeed(0);
    }

    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Elevator Left Speed", leftMotor.get());
    SmartDashboard.putNumber("Elevator Right Speed", rightMotor.get());
  }

  public void setSpeed(double speed) {
    leftMotor.set(speed);
  }

  public boolean getSwitch() {
    return limitSwitch.get();
  }

  public double getHeight() {
    return leftMotor.getEncoder().getPosition();
  }

  public PIDController getController() {
    return heightController;
  }

  public Command stopMotors() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command manualSpeed(double speed) {
    return new InstantCommand(() -> setSpeed(speed));
  }
}
