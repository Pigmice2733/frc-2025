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

public class AlgaeIntake extends SubsystemBase {
  private SparkMax motor;

  private ShuffleboardTab tab;
  private ShuffleboardLayout entryList;
  private GenericEntry entry;

  public AlgaeIntake() {
    motor = new SparkMax(CANConfig.ALGAE_INTAKE, MotorType.kBrushless);

    motor.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    tab = ShuffleboardConfig.ARM_TAB;
    entryList = tab.getLayout("Intakes", BuiltInLayouts.kList).withSize(1, 3)
        .withPosition(1, 0);
    entry = entryList.add("Algae", 0).withPosition(0, 0).getEntry();
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    entry.setDouble(motor.get());
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command startMotor() {
    return new InstantCommand(() -> setSpeed(SystemConfig.ALGAE_INTAKE_SPEED));
  }
}
