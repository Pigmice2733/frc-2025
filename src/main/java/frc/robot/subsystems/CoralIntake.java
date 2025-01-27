package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class CoralIntake extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;

  private ShuffleboardTab tab;
  private ShuffleboardLayout entryList;
  private GenericEntry leftEntry, rightEntry;

  public CoralIntake() {
    leftMotor = new SparkMax(CANConfig.CORAL_LEFT, MotorType.kBrushless);
    rightMotor = new SparkMax(CANConfig.CORAL_RIGHT, MotorType.kBrushless);

    leftMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    tab = ShuffleboardConfig.ARM_TAB;
    entryList = tab.getLayout("Intakes", BuiltInLayouts.kList).withSize(1, 3)
        .withPosition(1, 0);
    leftEntry = entryList.add("Coral Left", 0).withPosition(0, 1).getEntry();
    rightEntry = entryList.add("Coral Right", 0).withPosition(0, 2).getEntry();
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    leftEntry.setDouble(leftMotor.get());
    rightEntry.setDouble(rightMotor.get());
  }

  public void setSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command startMotor() {
    return new InstantCommand(() -> setSpeed(SystemConfig.CORAL_INTAKE_SPEED));
  }
}
