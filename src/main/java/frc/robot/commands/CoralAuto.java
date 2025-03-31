package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class CoralAuto extends SequentialCommandGroup {
  /**
   * Autonomous command that scores a Coral on the Reef L4, starting from the
   * center of the field.
   * 
   * @param dvt  drivetrain subsystem
   * @param vis  vision subsystem
   * @param elv  elevator subsystem
   * @param piv  pivot subsystem
   * @param gbr  grabber subsystem
   * @param ctlr XBox controller
   */
  public CoralAuto(Drivetrain dvt, Vision vis, Elevator elv, Pivot piv, Grabber gbr, CommandXboxController ctlr) {
    addCommands(
        Commands.parallel(
            RobotContainer.getDriveVisionCommand(dvt, vis, ctlr, Units.inchesToMeters(15.5), Units.inchesToMeters(8.2),
                0),
            new SetArmPosition(elv, piv, ArmPosition.SCORE_L4)).withTimeout(8),
        Commands.waitSeconds(1.5),
        gbr.runReverse(),
        Commands.waitSeconds(1.5),
        gbr.stopMotor(),
        new SetArmPosition(elv, piv, ArmPosition.STOW));
  }
}
