// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
 
  public static class ShuffleboardConfig {
    public static final ShuffleboardTab DRIVETRAIN_TAB = Shuffleboard.getTab("Drivetrain & Vision");
    public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm & Elevator");
    public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");
  }

  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 1.0;
    public static final double MAX_TURN_SPEED = 1.0;
    public static final PIDConstants DRIVE_PID = new PIDConstants(0.5, 0, 0);
    public static final PIDConstants TURN_PID = new PIDConstants(0.5, 0, 0);
  }

  public static class AutoConfig {
    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(0, 0, null, 0);
  }
}


