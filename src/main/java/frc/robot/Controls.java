package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConfig;

public class Controls {
    private CommandXboxController driver;

    // If a value from a joystick is less than this, it will return 0.
    private double threshold = Constants.AXIS_THRESHOLD;

    private boolean slowmode;

    public Controls(CommandXboxController driver) {
        this.driver = driver;

        setSlowmode(false);
    }

    /**
     * Returns the left joystick's Y-axis value.
     */
    public double getDriveSpeedY() {
        double joystickY = MathUtil.applyDeadband(driver.getLeftY(), threshold);

        return joystickY * DrivetrainConfig.MAX_DRIVE_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
    }

    /**
     * Returns the left joystick's X-axis value.
     */
    public double getDriveSpeedX() {
        double joystickX = MathUtil.applyDeadband(driver.getLeftX(), threshold);

        return joystickX * DrivetrainConfig.MAX_DRIVE_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
    }

    /**
     * Returns the right joystick's X-axis value
     */
    public double getTurnSpeed() {
        double joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);

        return joystickTurn * DrivetrainConfig.MAX_TURN_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
    }

    public Command toggleSlowmode() {
        return new InstantCommand(slowmode ? () -> setSlowmode(true) : () -> setSlowmode(false));
    }

    public void setSlowmode(boolean slow) {
        slowmode = slow;
        SmartDashboard.putBoolean("Slowmode", slowmode);
    }

    public boolean getSlowmode() {
        return slowmode;
    }
}