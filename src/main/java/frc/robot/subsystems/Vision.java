package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private boolean hasTarget;
  private int targetID;

  private double[] robotArray, targetArray;
  private Pose2d botPose;
  private Pose2d targetPose;

  /** Finds and uses AprilTags and other vision targets. */
  public Vision() {
    robotArray = new double[6];
    targetArray = new double[6];
    botPose = new Pose2d();

    hasTarget = false;
    targetID = 0;
  }

  @Override
  public void periodic() {
    robotArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
        .getDoubleArray(robotArray);
    botPose = new Pose2d(robotArray[0], robotArray[1], new Rotation2d(Units.degreesToRadians(robotArray[5])));

    targetArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(targetArray);
    targetPose = new Pose2d(targetArray[2], targetArray[0], new Rotation2d(Units.degreesToRadians(targetArray[4])));

    targetID = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);

    hasTarget = LimelightHelpers.getTV("");

    updateEntries();
  }

  private void updateEntries() {
    if (hasTarget) {
      Constants.sendNumberToElastic("Robot Field X", botPose.getX(), 2);
      Constants.sendNumberToElastic("Robot Field Y", botPose.getY(), 2);
      Constants.sendNumberToElastic("Robot Field Angle", botPose.getRotation().getDegrees(), 1);

      Constants.sendNumberToElastic("Target X Offset", targetPose.getX(), 2);
      Constants.sendNumberToElastic("Target Y Offset", targetPose.getY(), 2);
      Constants.sendNumberToElastic("Target Angle Offset", targetPose.getRotation().getDegrees(), 2);
      Constants.sendNumberToElastic("Target ID", targetID, 0);
    } else {
      Constants.sendNumberToElastic("Robot Field X", 0, 0);
      Constants.sendNumberToElastic("Robot Field Y", 0, 0);
      Constants.sendNumberToElastic("Robot Field Angle", 0, 0);

      Constants.sendNumberToElastic("Target X Offset", 0, 0);
      Constants.sendNumberToElastic("Target Y Offset", 0, 0);
      Constants.sendNumberToElastic("Target Angle Offset", 0, 0);
      Constants.sendNumberToElastic("Target ID", targetID, 0);
    }
  }

  /** Returns true when there is a visible target. */
  public boolean hasTarget() {
    return hasTarget;
  }

  /** ID of the target identified, or -1 if none found. */
  public int getTargetID() {
    return targetID;
  }

  /** Position of the robot with respect to the blue alliance wall. */
  public Pose2d getRobotPose() {
    return hasTarget ? botPose : new Pose2d();
  }

  /** Position of the nearest target with respect to the robot. */
  public Pose2d getTargetPose() {
    return hasTarget ? targetPose : new Pose2d();
  }
}
