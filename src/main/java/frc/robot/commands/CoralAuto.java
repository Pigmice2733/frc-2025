package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.*;

public class CoralAuto extends SequentialCommandGroup {
  public CoralAuto(Drivetrain dvt, Vision vis, Elevator elv, Pivot pivot, Grabber gbr, CommandXboxController ctlr) {
    addCommands(
        Commands.parallel(new DriveAndTurn(dvt, vis, ctlr, Units.inchesToMeters(17), Units.inchesToMeters(8.2), 0),
            new SetArmPosition(elv, pivot, ArmPosition.SCORE_L4)).withTimeout(8),
        Commands.waitSeconds(1.5),
        gbr.runReverse(),
        Commands.waitSeconds(1.5),
        gbr.stopMotor(),
        new SetArmPosition(elv, pivot, ArmPosition.STOW));
  }
}
