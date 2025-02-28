package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ElevatorConfig;
import frc.robot.Constants.SensorConfig;

public class Elevator extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private DigitalInput limitSwitch;
  private PIDController pidController;
  private SysIdRoutine routine;
  private double motorSpeed;

  public Elevator() {
    leftMotor = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
    rightMotor = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);

    leftMotor.configure(
        new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake)
            .apply(new EncoderConfig().positionConversionFactor(ElevatorConfig.ELEVATOR_CONVERSION)
                .velocityConversionFactor(ElevatorConfig.ELEVATOR_CONVERSION / 60.0)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(
        new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake)
            .apply(new EncoderConfig().positionConversionFactor(ElevatorConfig.ELEVATOR_CONVERSION)
                .velocityConversionFactor(ElevatorConfig.ELEVATOR_CONVERSION / 60.0)),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = ElevatorConfig.ELEVATOR_PID;
    pidController.setTolerance(ElevatorConfig.ELEVATOR_TOLERANCE);

    routine = new SysIdRoutine(
        new Config(Velocity.ofRelativeUnits(0.8, VelocityUnit.combine(Volts, Seconds)), Volts.of(4), Seconds.of(15)),
        new Mechanism(this::setSpeeds, this::log, this));

    limitSwitch = new DigitalInput(SensorConfig.ELEVATOR_LIMIT_SWITCH_CHANNEL);

    motorSpeed = 0;

    Constants.sendNumberToElastic("Elevator Up-P", 0, 3);
    Constants.sendNumberToElastic("Elevator Down-P", 0, 3);
    Constants.sendNumberToElastic("Elevator I", 0, 3);
    Constants.sendNumberToElastic("Elevator D", 0, 3);
    Constants.sendNumberToElastic("Elevator Offset", 0, 3);
  }

  @Override
  public void periodic() {
    setSpeeds(calculate());

    if (getSwitch()) {
      leftMotor.getEncoder().setPosition(0);
      rightMotor.getEncoder().setPosition(0);
    }

    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Elevator Left Speed", leftMotor.get(), 2);
    Constants.sendNumberToElastic("Elevator Right Speed", rightMotor.get(), 2);
    Constants.sendNumberToElastic("Elevator Output", motorSpeed, 2);
    Constants.sendNumberToElastic("Elevator Left Position", leftMotor.getEncoder().getPosition(), 2);
    Constants.sendNumberToElastic("Elevator Right Position", rightMotor.getEncoder().getPosition(), 2);

    Constants.sendBooleanToElastic("Elevator Limit Switch", getSwitch());

    Constants.sendNumberToElastic("Elevator Setpoint", pidController.getSetpoint(), 2);

    // upP = SmartDashboard.getNumber("Elevator Up-P", 0);
    // downP = SmartDashboard.getNumber("Elevator Down-P", 0);
    // pidController.setI(SmartDashboard.getNumber("Elevator I", 0));
    // pidController.setD(SmartDashboard.getNumber("Elevator D", 0));
    // heightOffset = SmartDashboard.getNumber("Elevator Offset", 0);
  }

  public void setSpeeds(double speed) {
    motorSpeed = speed;

    if ((motorSpeed < 0 && getSwitch())) {
      System.out.println("CANNOT GO BELOW MINIMUM HEIGHT.");
      motorSpeed = 0;
    }
    if (motorSpeed > 0 && getHeight() >= ElevatorConfig.ELEVATOR_UPPER_LIMIT) {
      System.out.println("Cannot go above maximum height.");
      motorSpeed = 0;
    }

    leftMotor.set(motorSpeed);
    rightMotor.set(motorSpeed);
  }

  public boolean getSwitch() {
    // negated because limit switch is wired for NC
    return !limitSwitch.get();
  }

  public double getHeight() {
    return leftMotor.getEncoder().getPosition();
  }

  public void setSetpoint(double height) {
    // use different p-values up and down
    if (height > getHeight()) {
      pidController.setP(ElevatorConfig.ELEVATOR_P_UP);
    } else {
      pidController.setP(ElevatorConfig.ELEVATOR_P_DOWN);
    }

    // prevent setpoint from being out of range
    if (height < 0) {
      pidController.setSetpoint(0);
    } else if (height > ElevatorConfig.ELEVATOR_UPPER_LIMIT) {
      pidController.setSetpoint(ElevatorConfig.ELEVATOR_UPPER_LIMIT);
    } else {
      pidController.setSetpoint(height);
    }
  }

  public void changeSetpoint(double delta) {
    if (delta != 0) {
      setSetpoint(getHeight() + delta);
    }
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  /** Returns the calculated output based on the current height and velocity. */
  public double calculate() {
    return pidController.calculate(getHeight()) + 0.05;
  }

  public Command stopMotors() {
    return new InstantCommand(() -> setSpeeds(0));
  }

  public Command manualSpeed(DoubleSupplier speed) {
    return Commands.run(() -> setSpeeds(speed.getAsDouble()), this);
  }

  public void setMotorSpeed(double motorSpeed) {
    this.motorSpeed = motorSpeed;
  }

  public double getMotorSpeed() {
    return motorSpeed;
  }

  private void log(SysIdRoutineLog log) {
    System.out.println("logging");
    log.motor("left")
        .voltage(Voltage.ofRelativeUnits(leftMotor.get() * 12, Volts))
        .linearPosition(Distance.ofRelativeUnits(leftMotor.getEncoder().getPosition(), Inches))
        .linearVelocity(LinearVelocity.ofRelativeUnits(leftMotor.getEncoder().getVelocity(), InchesPerSecond));
    log.motor("right")
        .voltage(Voltage.ofRelativeUnits(rightMotor.get() * 12, Volts))
        .linearPosition(Distance.ofRelativeUnits(rightMotor.getEncoder().getPosition(), Inches))
        .linearVelocity(LinearVelocity.ofRelativeUnits(rightMotor.getEncoder().getVelocity(), InchesPerSecond));
  }

  public Command sysIdQuasistatic(Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(Direction direction) {
    return routine.dynamic(direction);
  }

  private void setSpeeds(Voltage voltage) {
    setSpeeds(voltage.magnitude() / 12);
    System.out.println("setting motor speeds by voltage");
  }
}
