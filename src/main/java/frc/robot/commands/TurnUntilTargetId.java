package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnUntilTargetId extends Command {
  private int target;
  private double speed;
  private Vision vsn;
  private Drivetrain dvt;

  /**
   * Turns at the given constant speed until the camera sees the given target ID.
   * 
   * @param target     ID of the desired target
   * @param speed      constant speed for turning (positive is CCW)
   * @param vision     vision subsystem
   * @param drivetrain drivetrain subsystem
   */
  public TurnUntilTargetId(int target, double speed, Vision vision, Drivetrain drivetrain) {
    this.target = target;
    this.speed = speed;
    vsn = vision;
    dvt = drivetrain;

    addRequirements(vision, drivetrain);
  }

  @Override
  public void execute() {
    dvt.drive(0, 0, Units.degreesToRadians(speed));
  }

  @Override
  public void end(boolean interrupted) {
    dvt.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return vsn.getTargetID() == target;
  }

}
