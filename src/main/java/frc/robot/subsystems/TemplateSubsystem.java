package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class TemplateSubsystem extends SubsystemBase {
  private SparkMax motor = new SparkMax(0, MotorType.kBrushless);

  private ShuffleboardTab tab = ShuffleboardConfig.DRIVETRAIN_TAB;
  private ShuffleboardLayout entryList;
  private GenericEntry entry;

  public TemplateSubsystem() {
    entryList = tab.getLayout("Subsystem", BuiltInLayouts.kList).withSize(1, 4)
        .withPosition(1, 0);
    entry = entryList.add("Name", 0).withPosition(0, 0).getEntry();
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    entry.setDouble(motor.get());
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setMotorSpeed(0));
  }

  public Command startMotor() {
    return new InstantCommand(() -> setMotorSpeed(1));
  }
}
