package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.SystemConfig;

public class Controls {
  private CommandXboxController driver, operator;

  /** If a value from a joystick is less than this, it will return 0. */
  private double threshold = Constants.AXIS_THRESHOLD;

  private boolean slowmode;

  public Controls(CommandXboxController driver, CommandXboxController operator) {
    this.driver = driver;
    this.operator = operator;

    setSlowmode(false);
  }

  /**
   * Returns the left joystick's Y-axis value, corresponding to the robot's
   * X-direction speed.
   */
  public double getDriveSpeedX() {
    double joystickY = MathUtil.applyDeadband(driver.getLeftY(), threshold);

    return joystickY * DrivetrainConfig.MAX_DRIVE_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
  }

  /**
   * Returns the left joystick's X-axis value, corresponding to the robot's
   * Y-direction speed.
   */
  public double getDriveSpeedY() {
    double joystickX = MathUtil.applyDeadband(driver.getLeftX(), threshold);

    return joystickX * DrivetrainConfig.MAX_DRIVE_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
  }

  /**
   * Returns the right joystick's X-axis value, corresponding to the robot's
   * rotational speed.
   */
  public double getTurnSpeed() {
    double joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);

    return joystickTurn * DrivetrainConfig.MAX_TURN_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
  }

  public Command toggleSlowmode() {
    return new InstantCommand(slowmode ? () -> setSlowmode(false) : () -> setSlowmode(true));
  }

  public void setSlowmode(boolean slow) {
    slowmode = slow;
    SmartDashboard.putBoolean("Slowmode", slowmode);
  }

  public boolean getSlowmode() {
    return slowmode;
  }

  public double getElevatorSpeed() {
    double joystickSpeed = MathUtil.applyDeadband(operator.getLeftY(), threshold);

    return joystickSpeed * SystemConfig.MAX_ELEVATOR_SPEED;
  }

  public double getPivotSpeed() {
    double joystickSpeed = MathUtil.applyDeadband(operator.getRightY(), threshold);

    return joystickSpeed * SystemConfig.MAX_PIVOT_SPEED;
  }

  public double getShooterSpeed() {
    double joystickSpeed = MathUtil.applyDeadband(operator.getRightTriggerAxis(), threshold)
        - MathUtil.applyDeadband(operator.getLeftTriggerAxis(), threshold);

    return joystickSpeed * SystemConfig.MAX_SHOOTER_SPEED;
  }
}