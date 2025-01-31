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

public class Pivot extends SubsystemBase {
  private SparkMax motor;
  private PIDController motorController;

  public Pivot() {
    motor = new SparkMax(CANConfig.PIVOT, MotorType.kBrushless);
    motor.configure(
        new SparkMaxConfig().inverted(false)
            .apply(new AbsoluteEncoderConfig().positionConversionFactor(SystemConfig.PIVOT_CONVERSION)),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    motor.getEncoder().setPosition(0);

    motorController = SystemConfig.PIVOT_PID;
    motorController.setTolerance(SystemConfig.PIVOT_TOLERANCE);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Pivot Motor Speed", motor.get());
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public PIDController getController() {
    return motorController;
  }

  public double getAngle() {
    return motor.getEncoder().getPosition();
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command startMotor() {
    return new InstantCommand(() -> setSpeed(SystemConfig.PIVOT_SPEED));
  }
}
