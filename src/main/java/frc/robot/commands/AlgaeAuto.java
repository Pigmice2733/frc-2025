package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.*;

public class AlgaeAuto extends SequentialCommandGroup {
  /**
   * Autonomous command that intakes an Algae from the Reef, starting from the
   * center of the field.
   * 
   * @param dvt  drivetrain subsystem
   * @param vis  vision subsystem
   * @param elv  elevator subsystem
   * @param piv  pivot subsystem
   * @param gbr  grabber subsystem
   * @param shr  shooter subsystem
   * @param ctlr Xbox controller
   */
  public AlgaeAuto(Drivetrain dvt, Vision vis, Elevator elv, Pivot piv, Grabber gbr, Shooter shr,
      CommandXboxController ctlr) {
    addCommands(
        Commands.parallel(
            new DriveToTarget(dvt, vis, ctlr, Units.inchesToMeters(17), 0, 0),
            new SetArmPosition(elv, piv, ArmPosition.ALGAE_L2),
            Commands.runOnce(() -> shr.setPivotPositionSetpoint(ShooterConfig.PIVOT_NET_ANGLE))).withTimeout(10),
        gbr.runForward(),
        new IntakeAlgae(shr),
        Commands.waitSeconds(2),
        gbr.stopMotor(),
        shr.stopMotors());
  }
}
