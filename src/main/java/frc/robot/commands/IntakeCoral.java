package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CoralManipulator;

public class IntakeCoral extends SequentialCommandGroup {
  /** Intakes a coral from the human player station. */
  public IntakeCoral(CoralManipulator coral) {
    addCommands(
        coral.intake(),
        new WaitUntilCommand(coral::hasCoral),
        coral.stopMotor());
    addRequirements(coral);
  }
}
