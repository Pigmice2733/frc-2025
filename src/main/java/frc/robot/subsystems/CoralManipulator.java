package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class CoralManipulator extends SubsystemBase {
  private SparkMax motor;

  public CoralManipulator() {
    motor = new SparkMax(CANConfig.CORAL_GRABBER, MotorType.kBrushless);

    motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Coral Intake Motor Speed", motor.get(), 2);
  }

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command outtake() {
    return new InstantCommand(() -> setSpeed(SystemConfig.CORAL_INTAKE_SPEED));
  }

  public Command intake() {
    return new InstantCommand(() -> setSpeed(-SystemConfig.CORAL_INTAKE_SPEED));
  }
}
