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
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
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
import frc.robot.Constants.ArmConfig;
import frc.robot.Constants.CANConfig;

public class Pivot extends SubsystemBase {
  private SparkMax motor;
  private PIDController pidController;
  private ArmFeedforward ff;
  private SysIdRoutine routine;
  private double motorSpeed;

  public Pivot() {
    motor = new SparkMax(CANConfig.PIVOT, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake)
        .apply(new AbsoluteEncoderConfig().positionConversionFactor(ArmConfig.PIVOT_CONVERSION).inverted(true)),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = ArmConfig.PIVOT_PID;
    pidController.setTolerance(ArmConfig.PIVOT_TOLERANCE);

    ff = ArmConfig.PIVOT_FEEDFORWARD;

    routine = new SysIdRoutine(
        new Config(Velocity.ofRelativeUnits(2, VelocityUnit.combine(Volts, Seconds)), Volts.of(5), Seconds.of(15)),
        new Mechanism(this::setSpeed, this::log, this));

    motorSpeed = 0;
  }

  @Override
  public void periodic() {
    setSpeed(calculate());
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Pivot Motor Speed", motor.get(), 2);
    Constants.sendNumberToElastic("Target Pivot Motor Speed", motorSpeed, 2);
    Constants.sendNumberToElastic("Pivot Angle", getAngle(), 3);
  }

  public void setSpeed(double speed) {
    motorSpeed = speed;

    if (getAngle() <= ArmConfig.PIVOT_LOWER_LIMIT && motorSpeed < 0) {
      motorSpeed = 0;
      System.out.println("PIVOT LOWER STOP, Angle = " + getAngle());
    }
    if (getAngle() >= ArmConfig.PIVOT_UPPER_LIMIT && motorSpeed > 0) {
      motorSpeed = 0;
      System.out.println("PIVOT UPPER STOP, Angle = " + getAngle());
    }

    motor.set(motorSpeed);
  }

  public void setSetpoint(double angle) {
    pidController.setSetpoint(angle);
  }

  public void changeSetpoint(double delta) {
    pidController.setSetpoint(pidController.getSetpoint() + delta);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  /** Returns the calculated output based on the current angle and velocity. */
  public double calculate() {
    return pidController.calculate(getAngle())
        + ff.calculate(Units.degreesToRadians(getAngle()), motor.getAbsoluteEncoder().getVelocity());
  }

  public double getAngle() {
    double position = motor.getAbsoluteEncoder().getPosition();
    position += ArmConfig.PIVOT_ANGLE_OFFSET;
    return position < 0 ? position + 360 : position;
  }

  public Command stopMotor() {
    return new InstantCommand(() -> setSpeed(0));
  }

  public Command manualSpeed(DoubleSupplier speed) {
    return Commands.run(() -> setSpeed(speed.getAsDouble()), this);
  }

  public void setMotorSpeed(double motorSpeed) {
    this.motorSpeed = motorSpeed;
  }

  public double getMotorSpeed() {
    return motorSpeed;
  }

  private void log(SysIdRoutineLog log) {
    System.out.println("logging");
    log.motor("pivot")
        .voltage(Voltage.ofRelativeUnits(motor.get() * 12, Volts))
        .linearPosition(Distance.ofRelativeUnits(motor.getEncoder().getPosition(),
            Inches))
        .linearVelocity(LinearVelocity.ofRelativeUnits(motor.getEncoder().getVelocity(), InchesPerSecond));
  }

  public Command sysIdQuasistatic(Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(Direction direction) {
    return routine.dynamic(direction);
  }

  private void setSpeed(Voltage voltage) {
    System.out.println("setting motor speed by voltage");
    setSpeed(voltage.magnitude() / 12);
  }
}
