package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PivotPosition;
import frc.robot.Constants.SystemConfig;
import frc.robot.commands.SetPivotPosition;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Pivot;

public class ScoreCoralLow extends SequentialCommandGroup {
  public ScoreCoralLow(CoralManipulator coral, Pivot arm) {
    addCommands(
        new SetPivotPosition(arm, PivotPosition.SCORE_L1),
        coral.outtake(),
        new WaitCommand(SystemConfig.CORAL_OUTTAKE_TIME),
        coral.stopMotor());
    addRequirements(coral, arm);
  }
}
