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

public class Elevator extends SubsystemBase {
  private SparkMax frontMotor, backMotor;

  private ShuffleboardTab tab;
  private ShuffleboardLayout entryList;
  private GenericEntry frontEntry, backEntry;

  public Elevator() {
    frontMotor = new SparkMax(CANConfig.ELEVATOR_FRONT, MotorType.kBrushless);
    backMotor = new SparkMax(CANConfig.ELEVATOR_BACK, MotorType.kBrushless);

    frontMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    backMotor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    tab = ShuffleboardConfig.ARM_TAB;
    entryList = tab.getLayout("Elevator", BuiltInLayouts.kList).withSize(1, 3)
        .withPosition(1, 0);
    frontEntry = entryList.add("Elevator Front", 0).withPosition(0, 0).getEntry();
    backEntry = entryList.add("Elevator Back", 0).withPosition(0, 1).getEntry();
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    frontEntry.setDouble(frontMotor.get());
    backEntry.setDouble(backMotor.get());
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
