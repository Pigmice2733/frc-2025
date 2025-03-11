package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnUntilTargetId extends Command {
  int target;
  double speed;
  Vision vision;
  Drivetrain drivetrain;

  public TurnUntilTargetId(int target, double speed, Vision vision, Drivetrain drivetrain) {
    this.target = target;
    this.vision = vision;
    this.speed = speed;
    this.drivetrain = drivetrain;
    addRequirements(vision, drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(0, 0, Units.degreesToRadians(speed));
  }

  @Override
  public boolean isFinished() {
    return vision.getTargetID() == target;
  }

}
