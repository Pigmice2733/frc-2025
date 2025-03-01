package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.GrabberWheel;
import frc.robot.subsystems.CoralGrabber;

public class IntakeCoral extends SequentialCommandGroup {
  /** Intakes a coral from the human player station. */
  public IntakeCoral(CoralGrabber coral, GrabberWheel grabber) {
    addCommands(
        coral.intake(),
        grabber.runReverse(),
        new WaitUntilCommand(coral::hasCoral),
        coral.stopMotor(),
        grabber.stopMotor());
    addRequirements(coral);
  }
}
