package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.CoralManipulator;

public class ScoreCoral extends SequentialCommandGroup {
  /** Scores a coral on the reef. */
  public ScoreCoral(CoralManipulator coral) {
    addCommands(
        coral.outtake(),
        new WaitCommand(SystemConfig.CORAL_OUTTAKE_TIME),
        coral.stopMotor());
    addRequirements(coral);
  }
}
