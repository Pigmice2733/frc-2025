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

public class AlgaeGrabber extends SubsystemBase {
  private SparkMax motor;

  public AlgaeGrabber() {
    motor = new SparkMax(CANConfig.ALGAE_GRABBER, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Algae Grabber Motor", motor.get(), 2);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command runForward() {
    return new InstantCommand(() -> setSpeed(ArmConfig.GRABBER_SPEED));
  }

  public Command runReverse() {
    return new InstantCommand(() -> setSpeed(-1 * ArmConfig.GRABBER_SPEED));
  }
}