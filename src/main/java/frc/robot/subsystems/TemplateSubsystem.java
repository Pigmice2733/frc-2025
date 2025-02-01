package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TemplateSubsystem extends SubsystemBase {
  private SparkMax motor = new SparkMax(0, MotorType.kBrushless);
  private PIDController motorController;

  public TemplateSubsystem() {
    motor = new SparkMax(0, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    motorController = new PIDController(0, 0, 0);
    motorController.setTolerance(0);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Motor Speed", motor.get());
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setMotorSpeed(0), this);
  }

  public Command startMotor() {
    return new InstantCommand(() -> setMotorSpeed(1), this);
  }

  public PIDController getController() {
    return motorController;
  }
}
