package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class CoralManipulator extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;

  public CoralManipulator() {
    leftMotor = new SparkMax(CANConfig.CORAL_LEFT, MotorType.kBrushless);
    rightMotor = new SparkMax(CANConfig.CORAL_RIGHT, MotorType.kBrushless);

    leftMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    rightMotor.configure(new SparkMaxConfig().inverted(true).follow(CANConfig.CORAL_LEFT),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Coral Intake Left Motor Speed", leftMotor.get());
    SmartDashboard.putNumber("Coral Intake Right Motor Speed", rightMotor.get());
  }

  private void setSpeed(double speed) {
    leftMotor.set(speed);
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
