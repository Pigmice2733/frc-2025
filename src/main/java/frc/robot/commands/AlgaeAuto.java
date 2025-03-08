package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.ShooterConfig;
import frc.robot.subsystems.*;

public class AlgaeAuto extends SequentialCommandGroup {
  public AlgaeAuto(Drivetrain dvt, Vision vis, Elevator elv, Pivot pivot, Grabber gbr, Shooter shr,
      CommandXboxController ctlr) {
    addCommands(Commands.parallel(
        new DriveToTarget(dvt, vis, ctlr, Units.inchesToMeters(17), 0, 0),
        new SetArmPosition(elv, pivot, ArmPosition.ALGAE_L2),
        Commands.runOnce(() -> shr.setPivotPositionSetpoint(ShooterConfig.PIVOT_NET_ANGLE))).withTimeout(10),
        gbr.runForward(),
        new IntakeAlgae(shr),
        Commands.waitSeconds(2),
        gbr.stopMotor(),
        shr.stopMotors());
  }
}
