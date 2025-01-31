package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SystemConfig;
import frc.robot.subsystems.AlgaeGrabber;

public class AlgaeFromReef extends SequentialCommandGroup {
  /** Removes an algae from the reef. */
  public AlgaeFromReef(AlgaeGrabber grabber) {
    addCommands(
        grabber.runMotor(),
        new WaitCommand(SystemConfig.GRABBER_TIME),
        grabber.stopMotor());
    addRequirements(grabber);
  }
}