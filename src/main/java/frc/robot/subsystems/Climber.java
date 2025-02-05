package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.SystemConfig;

public class Climber extends SubsystemBase {
    private Servo motor;

    public Climber() {
        motor = new Servo(CANConfig.CLIMBER);
    }

    @Override
    public void periodic() {
        updateEntries();
    }

    private void updateEntries() {
        SmartDashboard.putNumber("Climber Position", getPosition());
    }

    public void setPosition(double position) {
        motor.set(position);
    }

    private void setDefaultPosition() {
        motor.set(SystemConfig.CLIMBER_DEFAULT_POSITION);
    }

    private void setClimbingPosition() {
        motor.set(SystemConfig.CLIMBER_CLIMB_POSITION);
    }

    public double getPosition() {
        return motor.get();
    }

    public Command defaultPosition() {
        return new InstantCommand(this::setDefaultPosition);
    }

    public Command climbPosition() {
        return new InstantCommand(this::setClimbingPosition);
    }
}
