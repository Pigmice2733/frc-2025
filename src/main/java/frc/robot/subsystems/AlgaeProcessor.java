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

public class AlgaeProcessor extends SubsystemBase {
  private SparkMax motor;

  public AlgaeProcessor() {
    motor = new SparkMax(CANConfig.ALGAE_INTAKE, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Algae Intake Motor", motor.get());
  }

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command outtake() {
    return new InstantCommand(() -> setSpeed(SystemConfig.PROCESSOR_SPEED));
  }

  public Command intake() {
    return new InstantCommand(() -> setSpeed(-SystemConfig.PROCESSOR_SPEED));
  }
}