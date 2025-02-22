package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private DigitalInput limitSwitch;
  private PIDController heightController;
  private double motorSpeed;

  public Elevator() {
    leftMotor = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
    rightMotor = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);

    leftMotor.configure(
        new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake)
            .apply(new EncoderConfig().positionConversionFactor(SystemConfig.ELEVATOR_CONVERSION)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(
        new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake)
            .apply(new EncoderConfig().positionConversionFactor(SystemConfig.ELEVATOR_CONVERSION)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    heightController = SystemConfig.ELEVATOR_PID;
    heightController.setTolerance(SystemConfig.ELEVATOR_TOLERANCE);

    limitSwitch = new DigitalInput(CANConfig.LIMIT_SWITCH_CHANNEL);

    Constants.sendNumberToElastic("Elevator P", 0, 0);
    Constants.sendNumberToElastic("Elevator I", 0, 0);
    Constants.sendNumberToElastic("Elevator D", 0, 0);

    motorSpeed = 0;
  }

  @Override
  public void periodic() {
    setSpeeds(motorSpeed);

    if (getSwitch()) {
      leftMotor.getEncoder().setPosition(0);
      rightMotor.getEncoder().setPosition(0);
    }

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Elevator Left Speed", leftMotor.get(), 2);
    Constants.sendNumberToElastic("Elevator Right Speed", rightMotor.get(), 2);
    Constants.sendNumberToElastic("Elevator Left Position", leftMotor.getEncoder().getPosition(), 2);
    Constants.sendNumberToElastic("Elevator Right Position", rightMotor.getEncoder().getPosition(), 2);
    Constants.sendBooleanToElastic("Elevator Limit Switch", getSwitch());

    heightController = new PIDController(SmartDashboard.getNumber("Elevator P", 0),
        SmartDashboard.getNumber("Elevator I", 0),
        SmartDashboard.getNumber("Elevator D", 0));
  }

  public void setSpeeds(double speed) {
    motorSpeed = speed;
    if ((leftMotor.get() < 0 && getSwitch())) {
      System.out.println("CANNOT GO BELOW MINIMUM HEIGHT.");
      motorSpeed = 0;
    }
    if (leftMotor.get() > 0 && getHeight() >= SystemConfig.ELEVATOR_UPPER_LIMIT) {
      System.out.println("Cannot go above maximum height.");
      motorSpeed = 0;
    }

    leftMotor.set(motorSpeed);
    rightMotor.set(motorSpeed);
  }

  public boolean getSwitch() {
    // Negated because limit switch is wired for NC
    return !limitSwitch.get();
  }

  public double getHeight() {
    return leftMotor.getEncoder().getPosition();
  }

  public PIDController getController() {
    return heightController;
  }

  public Command stopMotors() {
    return new InstantCommand(() -> setSpeeds(0));
  }

  public Command manualSpeed(DoubleSupplier speed) {
    return Commands.run(() -> setSpeeds(speed.getAsDouble()), this);
  }
}
