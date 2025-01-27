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

public class Elevator extends SubsystemBase {
  private SparkMax frontMotor, backMotor;

  public Elevator() {
    frontMotor = new SparkMax(CANConfig.ELEVATOR_FRONT, MotorType.kBrushless);
    backMotor = new SparkMax(CANConfig.ELEVATOR_BACK, MotorType.kBrushless);

    frontMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    backMotor.configure(new SparkMaxConfig().inverted(false).follow(CANConfig.ELEVATOR_FRONT),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    SmartDashboard.putNumber("Elevator Front Speed", frontMotor.get());
    SmartDashboard.putNumber("Elevator Back Speed", backMotor.get());
  }

  public void setSpeed(double speed) {
    frontMotor.set(speed);
    backMotor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command startMotor() {
    return new InstantCommand(() -> setSpeed(SystemConfig.ELEVATOR_SPEED));
  }
}
