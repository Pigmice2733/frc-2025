package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class CoralManipulator extends SubsystemBase {
  private SparkMax motor;
  private DigitalInput beamBreak;

  public CoralManipulator() {
    motor = new SparkMax(CANConfig.CORAL_GRABBER, MotorType.kBrushless);

    motor.configure(new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    beamBreak = new DigitalInput(Constants.SensorConfig.CORAL_BEAM_BREAK_CHANNEL);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Coral Intake Motor Speed", motor.get(), 2);
    Constants.sendBooleanToElastic("Coral Beam Break Raw", beamBreak.get());
    Constants.sendBooleanToElastic("Has Coral", !beamBreak.get());
  }

  public boolean hasCoral() {
    return !beamBreak.get();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command outtake() {
    System.out.println("outtaking");
    return new InstantCommand(() -> setSpeed(ArmConfig.CORAL_OUTTAKE_SPEED));
  }

  public Command intake() {
    System.out.println("intaking");
    return new InstantCommand(() -> setSpeed(ArmConfig.CORAL_INTAKE_SPEED));
  }
}
