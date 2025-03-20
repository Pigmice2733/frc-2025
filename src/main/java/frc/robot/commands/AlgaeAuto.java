package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.*;

public class AlgaeAuto extends SequentialCommandGroup {
  /**
   * Autonomous command that scores a Coral on L4 and intakes an Algae from the
   * Reef, starting from the center of the field.
   * 
   * @param dvt    drivetrain subsystem
   * @param vis    vision subsystem
   * @param elv    elevator subsystem
   * @param piv    pivot subsystem
   * @param gbr    grabber subsystem
   * @param shr    shooter subsystem
   * @param ctlr   XBox controller
   * @param height true for L3 Algae, false for L2
   */
  public AlgaeAuto(Drivetrain dvt, Vision vis, Elevator elv, Pivot piv, Grabber gbr, Shooter shr,
      CommandXboxController ctlr, boolean height) {
    addCommands(
        Commands.parallel(
            new DriveAndTurn(dvt, vis, ctlr, Units.inchesToMeters(17), Units.inchesToMeters(8.2), 0),
            new SetArmPosition(elv, piv, ArmPosition.SCORE_L4)).withTimeout(8),
        Commands.waitSeconds(1.5),
        gbr.runReverse(),
        Commands.waitSeconds(1.5),
        gbr.stopMotor(),
        Commands.parallel(
            new DrivePath(dvt, new Transform2d(0, -Units.inchesToMeters(8.2), new Rotation2d())),
            Commands.runOnce(() -> shr.setPivotPositionSetpoint(ShooterConfig.PIVOT_NET_ANGLE)),
            new SetArmPosition(elv, piv, height ? ArmPosition.ALGAE_L3 : ArmPosition.ALGAE_L2)).withTimeout(8),
        gbr.runForward(),
        new IntakeAlgae(shr),
        Commands.waitSeconds(1.5),
        gbr.stopMotor(),
        shr.stopMotors(),
        new SetArmPosition(elv, piv, ArmPosition.STOW));
  }
}
