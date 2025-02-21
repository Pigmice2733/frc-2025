// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double AXIS_THRESHOLD = 0.1;

  public static class CANConfig {
    public static final int ELEVATOR_LEFT = 14;
    public static final int ELEVATOR_RIGHT = 15;
    public static final int PIVOT = 16;
    public static final int ALGAE_GRABBER = 17;
    public static final int CORAL_LEFT = 18;
    public static final int CORAL_RIGHT = 19;
    public static final int SHOOTER_FLYWHEELS_LEFT = 20;
    public static final int SHOOTER_FLYWHEELS_RIGHT = 21;
    public static final int SHOOTER_PIVOT_LEFT = 22;
    public static final int SHOOTER_PIVOT_RIGHT = 23;
    public static final int INDEXER = 24;

    public static final int LIMIT_SWITCH_CHANNEL = 25;
  }

  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 10.0; // m/s
    public static final double MAX_TURN_SPEED = 200.0; // deg/s
    public static final double SLOWMODE_FACTOR = 0.4;
    public static final PIDConstants DRIVE_PID = new PIDConstants(0.5, 0, 0);
    public static final PIDConstants TURN_PID = new PIDConstants(0.5, 0, 0);
  }

  public static class AutoConfig {
  }

  public static class SystemConfig {
    public static final PIDController PIVOT_PID = new PIDController(0.1, 0, 0);
    public static final PIDController SHOOTER_PID = new PIDController(0.1, 0, 0);
    public static final PIDController ELEVATOR_PID = new PIDController(0.1, 0, 0);
    public static final double PIVOT_TOLERANCE = 2; // deg
    public static final double SHOOTER_TOLERANCE = 2; // deg
    public static final double ELEVATOR_TOLERANCE = 0.1; // m

    public static final double MAX_ELEVATOR_SPEED = 0.5;
    public static final double MAX_PIVOT_SPEED = 0.5;
    public static final double MAX_SHOOTER_SPEED = 0.5;
    public static final double GRABBER_SPEED = 0.5;
    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double FLYWHEEL_PROCESSOR_SPEED = 0.2;
    public static final double FLYWHEEL_NET_SPEED = 0.8;
    public static final double INDEXER_SPEED = 0.5;

    public static final double PIVOT_CONVERSION = 1;
    public static final double ELEVATOR_CONVERSION = 1;
    public static final double SHOOTER_CONVERSION = 1;

    // seconds
    public static final double SHOOTER_SPINUP_TIME = 2.0;
    public static final double SHOOTER_SHOOT_TIME = 2.0;
    public static final double SHOOTER_INTAKE_TIME = 0.5;
    public static final double CORAL_INTAKE_TIME = 1.0;
    public static final double CORAL_OUTTAKE_TIME = 1.0;
    public static final double GRABBER_TIME = 3.0;

    public static final double CLIMBER_DEFAULT_POSITION = 0;
    public static final double CLIMBER_CLIMB_POSITION = 0.5;

    public static final double ELEVATOR_UPPER_LIMIT = Units.inchesToMeters(21.5);
    public static final double PIVOT_LOWER_LIMIT = 10; // deg
    public static final double PIVOT_UPPER_LIMIT = 270; // deg
    public static final double SHOOTER_LOWER_LIMIT = -12; // deg
    public static final double SHOOTER_UPPER_LIMIT = 90; // deg
  }

  public static enum ElevatorPosition {
    HUMAN_PLAYER(120, 0), SCORE_L1(260, 0), SCORE_L2(225, 0.15), SCORE_L3(225, 0.35), SCORE_L4(200, 0.5),
    REEF_L2(210, 0.2), REEF_L3(210, 0.4), STOW(0, 0), CLIMB(0, 0.1);

    private double pivotAngle, elevatorHeight;

    private ElevatorPosition(double pivot, double elevator) {
      pivotAngle = pivot;
      elevatorHeight = elevator;
    }

    public double getPivotAngle() {
      return pivotAngle;
    }

    public double getElevatorHeight() {
      return elevatorHeight;
    }
  }

  public static enum ShooterPosition {
    INTAKE(-8), PROCESSOR(0), NET(40), STOW(90);

    private double angle;

    private ShooterPosition(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }

  public static enum OperatorMode {
    NONE,
    CORAL,
    ELEVATOR,
    SHOOTER
  }

  public static void sendNumberToElastic(String name, double num, double places) {
    double newNum = Math.round(num * Math.pow(10, places)) / Math.pow(10, places);
    SmartDashboard.putNumber(name, newNum);
  }
}
