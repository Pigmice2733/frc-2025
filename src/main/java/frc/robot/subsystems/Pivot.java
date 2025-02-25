package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.MathUtil;
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
        .apply(new AbsoluteEncoderConfig().positionConversionFactor(ArmConfig.PIVOT_CONVERSION).inverted(true)),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = ArmConfig.PIVOT_PID;
    pidController.setTolerance(ArmConfig.PIVOT_TOLERANCE);

    motorSpeed = 0;
  }

  @Override
  public void periodic() {
    setSpeed(calculate());

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Pivot Motor Speed", motor.get(), 2);
    Constants.sendNumberToElastic("Target Pivot Motor Speed", motorSpeed, 2);
    Constants.sendNumberToElastic("Pivot Angle", getAngle(), 3);
  }

  public void setSpeed(double speed) {
    motorSpeed = speed;

    if (getAngle() <= ArmConfig.PIVOT_LOWER_LIMIT && motorSpeed < 0) {
      motorSpeed = 0;
      System.out.println("PIVOT LOWER STOP, Angle = " + getAngle());
    }
    if (getAngle() >= ArmConfig.PIVOT_UPPER_LIMIT && motorSpeed > 0) {
      motorSpeed = 0;
      System.out.println("PIVOT UPPER STOP, Angle = " + getAngle());
    }

    motorSpeed += 0.03 * MathUtil.applyDeadband(Math.sin(getAngle()), 0.1);
    motor.set(motorSpeed);
  }

  public void setSetpoint(double angle) {
    pidController.setSetpoint(angle);
  }

  public void changeSetpoint(double delta) {
    pidController.setSetpoint(pidController.getSetpoint() + delta);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  /** Returns the calculated output based on the current angle. */
  public double calculate() {
    return pidController.calculate(getAngle());
  }

  public double getAngle() {
    double position = motor.getAbsoluteEncoder().getPosition();
    position += ArmConfig.PIVOT_ANGLE_OFFSET;
    return position < 0 ? position + 360 : position;
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command manualSpeed(DoubleSupplier speed) {
    return Commands.run(() -> setSpeed(speed.getAsDouble()), this);
  }

  public void setMotorSpeed(double motorSpeed) {
    this.motorSpeed = motorSpeed;
  }

  public double getMotorSpeed() {
    return motorSpeed;
  }
}
