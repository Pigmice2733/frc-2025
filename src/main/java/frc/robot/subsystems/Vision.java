package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DrivetrainConfig;

public class Vision extends SubsystemBase {
  private boolean hasTarget;

  private double[] target;
  private Pose2d targetPose;

  /** Finds and uses AprilTags and other vision targets. */
  public Vision() {
    target = new double[6];
    targetPose = new Pose2d();

    hasTarget = false;
  }

  @Override
  public void periodic() {
    target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
        .getDoubleArray(target);

    // Position of the target with relation to the center of the robot.
    targetPose = new Pose2d(target[2] * -1.0 + target[4] * 0,
        target[0] + target[4] * 0,
        new Rotation2d(Units.degreesToRadians(target[4]) * -1)).plus(DrivetrainConfig.CAMERA_OFFSET);

    hasTarget = LimelightHelpers.getTV("");

    updateEntries();
  }

  private void updateEntries() {
    if (hasTarget) {
      Constants.sendNumberToElastic("Target X Offset", targetPose.getX(), 2);
      Constants.sendNumberToElastic("Target Y Offset", targetPose.getY(), 2);
      Constants.sendNumberToElastic("Target Angle Offset", targetPose.getRotation().getDegrees(), 1);
    } else {
      Constants.sendNumberToElastic("Target X Offset", 0, 0);
      Constants.sendNumberToElastic("Target Y Offset", 0, 0);
      Constants.sendNumberToElastic("Target Angle Offset", 0, 0);
    }
  }

  /** Returns true when there is a visible target. */
  public boolean hasTarget() {
    return hasTarget;
  }

  public Pose2d getTarget() {
    if (hasTarget)
      return targetPose;
    return new Pose2d();
  }
}
