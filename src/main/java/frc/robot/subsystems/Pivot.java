package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Pivot extends SubsystemBase {
  private SparkMax motor;
  private PIDController pidController;
  private double motorSpeed;

  public Pivot() {
    motor = new SparkMax(CANConfig.PIVOT, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake)
        .apply(new AbsoluteEncoderConfig().positionConversionFactor(SystemConfig.PIVOT_CONVERSION).inverted(true)),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = SystemConfig.PIVOT_PID;
    pidController.setTolerance(SystemConfig.PIVOT_TOLERANCE);

    Constants.sendNumberToElastic("Pivot P", 0, 0);
    Constants.sendNumberToElastic("Pivot I", 0, 0);
    Constants.sendNumberToElastic("Pivot D", 0, 0);

    motorSpeed = 0;
  }

  @Override
  public void periodic() {
    setSpeed(motorSpeed);

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Pivot Motor Speed", motor.get(), 2);
    Constants.sendNumberToElastic("Pivot Angle", getAngle(), 3);

    pidController = new PIDController(SmartDashboard.getNumber("Pivot P", 0),
        SmartDashboard.getNumber("Pivot I", 0),
        SmartDashboard.getNumber("Pivot D", 0));
  }

  public void setSpeed(double speed) {
    motorSpeed = speed;

    if (getAngle() <= SystemConfig.PIVOT_LOWER_LIMIT && motorSpeed < 0) {
      motorSpeed = 0;
    }
    if (getAngle() >= SystemConfig.PIVOT_UPPER_LIMIT && motorSpeed > 0) {
      motorSpeed = 0;
    }

    motor.set(motorSpeed);
  }

  public PIDController getController() {
    return pidController;
  }

  public double getAngle() {
    double position = motor.getAbsoluteEncoder().getPosition();
    position += SystemConfig.PIVOT_ANGLE_OFFSET;
    return position < 0 ? position + 360 : position;
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command manualSpeed(DoubleSupplier speed) {
    return Commands.run(() -> setSpeed(speed.getAsDouble()), this);
  }
}
