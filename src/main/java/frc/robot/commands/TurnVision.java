package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnVision extends Command {
  private Drivetrain drivetrain;
  private Vision vision;
  private double rOffset, setpoint, error, kP;

  // private PIDController rPID;
  // private PIDConstants pidConstants;
  private int id, counter;

  private Pose2d target, robotPose;

  /**
   * A command that drives the robot to the given position with respect to the
   * target. Tolerances have been set generously high to avoid spasmodic behavior,
   * but such a paradigm intended for precision will need lower tolerances.
   * 
   * @param drivetrain drivetrain subsystem
   * @param vision     vision subsystem
   * @param rOffset    target angle relative from tag
   */
  public TurnVision(Drivetrain drivetrain, Vision vision, double rOffset) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    this.rOffset = rOffset;

    setpoint = error = 0;
    counter = 0;

    Constants.sendNumberToElastic("Turn kP", 3.5, 2);

    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    // pidConstants = DrivetrainConfig.TURN_PID;

    // rPID = (PIDController) SmartDashboard.getData("Drivetrain Turn PID");
    // rPID = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    // rPID.setTolerance(DrivetrainConfig.TURN_POSITION_TOLERANCE,
    // DrivetrainConfig.TURN_VELOCITY_TOLERANCE);

    drivetrain.resetPose(new Pose2d());
    id = vision.getTargetID();
    getTargetSetpoint();

    kP = SmartDashboard.getNumber("Turn kP", 0);

    System.out.println("turn started");
  }

  @Override
  public void execute() {
    counter++;
    robotPose = drivetrain.getPose();

    if (vision.hasTarget() && vision.getTargetID() == id && counter >= 5) {
      getTargetSetpoint();
      counter = 0;
    }

    error = setpoint - robotPose.getRotation().getDegrees();

    drivetrain.driveField(0, 0, Units.degreesToRadians(error) * kP);

    // System.out.println("position " + robotPose.getRotation().getDegrees()
    // + " velocity " + calc);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveField(0, 0, 0);
    // drivetrain.getSwerve().lockPose();
    System.out.println("turn finished");
  }

  @Override
  public boolean isFinished() {
    return Math
        .abs(drivetrain.getSwerve().getFieldVelocity().omegaRadiansPerSecond) < DrivetrainConfig.TURN_VELOCITY_TOLERANCE
        &&
        (Math.abs(error) < DrivetrainConfig.TURN_POSITION_TOLERANCE || vision.getTargetID() != id);
  }

  private void getTargetSetpoint() {
    drivetrain.resetPose(new Pose2d());
    target = vision.getTargetPose();

    /* The PID controllers use the robot's pose, not the target pose. */
    setpoint = target.getRotation().getDegrees() + rOffset;

    Constants.sendNumberToElastic("Turn PID Setpoint", setpoint, 3);
  }
}
