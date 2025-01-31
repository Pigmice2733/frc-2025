package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PivotPosition;
import frc.robot.Constants.SystemConfig;
import frc.robot.commands.SetPivotPosition;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Pivot;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(CoralManipulator coral, Pivot arm) {
    addCommands(
        new SetPivotPosition(arm, PivotPosition.HUMAN_PLAYER),
        coral.intake(),
        new WaitCommand(SystemConfig.CORAL_INTAKE_TIME),
        coral.stopMotor());
    addRequirements(coral, arm);
  }
}
