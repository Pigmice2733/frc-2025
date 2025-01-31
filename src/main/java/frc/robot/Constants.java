// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

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
    public static final int ELEVATOR_FRONT = 0;
    public static final int ELEVATOR_BACK = 1;
    public static final int PIVOT = 2;
    public static final int ALGAE_INTAKE = 3;
    public static final int CORAL_LEFT = 4;
    public static final int CORAL_RIGHT = 5;
    public static final int SHOOTER_FLYWHEELS_LEFT = 6;
    public static final int SHOOTER_FLYWHEELS_RIGHT = 7;
    public static final int SHOOTER_PIVOT_LEFT = 8;
    public static final int SHOOTER_PIVOT_RIGHT = 9;
    public static final int INDEXER = 10;

    public static final int LIMIT_SWITCH_CHANNEL = 0;
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
    public static final double PIVOT_TOLERANCE = 2; // deg

    public static final double ELEVATOR_SPEED = 0.5;
    public static final double PIVOT_SPEED = 0.5;
    public static final double ALGAE_INTAKE_SPEED = 0.5;
    public static final double CORAL_INTAKE_SPEED = 0.5;
    public static final double SHOOTER_PIVOT_SPEED = 0.5;
    public static final double SHOOTER_FLYWHEEL_SPEED = 0.5;
    public static final double INDEXER_SPEED = 0.5;

    public static final double PIVOT_CONVERSION = 1;

    public static final double SHOOTER_SPINUP_TIME = 2.0;
    public static final double SHOOTER_SHOOT_TIME = 2.0;
    public static final double CORAL_INTAKE_TIME = 1.0;
    public static final double CORAL_OUTTAKE_TIME = 1.0;
  }

  public static enum PivotPosition {
    INITIAL(0), HUMAN_PLAYER(-30), SCORE_L4(20), SCORE_L3(30), SCORE_L1(80), PROCESSOR(120), REEF(45);

    private double angle;

    private PivotPosition(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }
}
