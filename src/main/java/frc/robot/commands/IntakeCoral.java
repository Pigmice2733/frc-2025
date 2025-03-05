package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Grabber;

public class IntakeCoral extends SequentialCommandGroup {
  /** Intakes a coral from the human player station. */
  public IntakeCoral(Grabber grabber) {
    addCommands(
        grabber.runForward(),
        new WaitUntilCommand(grabber::hasCoral),
        grabber.stopMotor());
    addRequirements(grabber);
  }
}
