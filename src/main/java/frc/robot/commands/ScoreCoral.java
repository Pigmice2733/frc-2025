package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.CoralManipulator;

public class ScoreCoral extends Command {
  private CoralManipulator coral;

  /** Scores a coral on the reef. */
  public ScoreCoral(CoralManipulator coral) {
    this.coral = coral;
    addRequirements(coral);
  }

  @Override
  public void initialize() {
    System.out.println("score init");
    coral.outtake();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("score end");
    coral.stopMotor();
  }

}
