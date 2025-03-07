package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Grabber extends SubsystemBase {
  private SparkMax motor;
  private DigitalInput beamBreak;
  private int ticks = 0;
  private boolean tickMode = true;
  private boolean tickModeRunning = true;

  public Grabber() {
    motor = new SparkMax(CANConfig.GRABBER, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().inverted(false).secondaryCurrentLimit(12), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    beamBreak = new DigitalInput(Constants.SensorConfig.CORAL_BEAM_BREAK_CHANNEL);
  }

  @Override
  public void periodic() {
    if (tickMode) {
      ticks++;
      if (ticks >= 10) {
        toggleMotor();
        ticks = 0;
        tickModeRunning = !tickModeRunning;
      }
    }

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Grabber Motor", motor.get(), 2);
    Constants.sendBooleanToElastic("Has Coral", hasCoral());
  }

  public boolean hasCoral() {
    return !beamBreak.get();
  }

  public void setSpeed(double speed, boolean tickMode) {
    this.tickMode = tickMode;
    motor.set(speed);
  }

  private void toggleMotor() {
    if (tickModeRunning) {
      setSpeed(0.1, true);
    } else {
      setSpeed(0, true);
    }
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0, true));
  }

  public Command runForward() {
    return new InstantCommand(() -> setSpeed(ArmConfig.GRABBER_SPEED, false));
  }

  public Command runReverse() {
    return new InstantCommand(() -> setSpeed(-1 * ArmConfig.GRABBER_SPEED, false));
  }
}