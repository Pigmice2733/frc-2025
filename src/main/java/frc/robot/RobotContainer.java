// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ElevatorPosition;
import frc.robot.Constants.OperatorMode;
import frc.robot.Constants.ShooterPosition;
import frc.robot.commands.AlgaeFromReef;
import frc.robot.commands.DriveJoysticks;
import frc.robot.commands.ElevatorControl;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.PivotControl;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.commands.TestDrive;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final AlgaeGrabber grabber;
  // private final AlgaeShooter shooter;
  private final CoralManipulator coral;
  private final Elevator elevator;
  private final Pivot pivot;

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private final Controls controls;

  private OperatorMode mode;
  public static ElevatorPosition elevPos;
  public static ShooterPosition shootPos;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);
    controls = new Controls(driver, operator);

    drivetrain = new Drivetrain();
    grabber = new AlgaeGrabber();
    // shooter = new AlgaeShooter();
    coral = new CoralManipulator();
    elevator = new Elevator();
    pivot = new Pivot();

    mode = OperatorMode.NONE;
    elevPos = ElevatorPosition.STOW;
    shootPos = ShooterPosition.STOW;

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DriveJoysticks(
        drivetrain,
        controls::getDriveSpeedX,
        controls::getDriveSpeedY,
        controls::getTurnSpeed));

    elevator.setDefaultCommand(new ElevatorControl(elevator, controls::getElevatorSpeed));
    pivot.setDefaultCommand(new PivotControl(pivot, controls::getPivotSpeed));
    // shooter.setDefaultCommand(shooter.manualSpeed(controls.getShooterSpeed()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // DRIVER
    driver.a().onTrue(drivetrain.reset());
    driver.y().onTrue(controls.toggleSlowmode());
    driver.x().onTrue(new TestDrive(drivetrain));
    // create vision commands once Vision subsystem exists

    // OPERATOR
    // operator.a().onTrue(new InstantCommand(() ->
    // changeMode(OperatorMode.SHOOTER)));
    // operator.povDown().and(() -> mode == OperatorMode.SHOOTER)
    // .onTrue(new SetShooterPosition(shooter, ShooterPosition.INTAKE));
    // operator.povRight().and(() -> mode == OperatorMode.SHOOTER)
    // .onTrue(new SetShooterPosition(shooter, ShooterPosition.PROCESSOR));
    // operator.povUp().and(() -> mode == OperatorMode.SHOOTER)
    // .onTrue(new SetShooterPosition(shooter, ShooterPosition.NET));
    // operator.povLeft().and(() -> mode == OperatorMode.SHOOTER)
    // .onTrue(new SetShooterPosition(shooter, ShooterPosition.STOW));
    // operator.leftBumper().and(() -> (shootPos == ShooterPosition.INTAKE))
    // .onTrue(new IntakeAlgae(shooter));
    // operator.leftBumper().and(() -> (shootPos == ShooterPosition.NET)).onTrue(new
    // ShootNet(shooter));
    // operator.leftBumper().and(() -> (shootPos == ShooterPosition.PROCESSOR))
    // .onTrue(new ShootProcessor(shooter));

    operator.b().onTrue(new InstantCommand(() -> changeMode(OperatorMode.ELEVATOR)));
    operator.povDown().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot,
            ElevatorPosition.STOW));
    operator.povLeft().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot,
            ElevatorPosition.HUMAN_PLAYER));
    operator.povRight().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ElevatorPosition.ALGAE_L2));
    operator.povUp().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ElevatorPosition.ALGAE_L3));
    // operator.rightBumper().and(() -> (elevPos == ElevatorPosition.HUMAN_PLAYER))
    // .onTrue(new IntakeCoral(coral));
    // operator.rightBumper().and(() -> (elevPos == ElevatorPosition.ALGAE_L2
    // || elevPos == ElevatorPosition.ALGAE_L3)).onTrue(new AlgaeFromReef(grabber));

    operator.x().onTrue(new InstantCommand(() -> changeMode(OperatorMode.REEF)));
    operator.povDown().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ElevatorPosition.SCORE_L1));
    operator.povLeft().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ElevatorPosition.SCORE_L2));
    operator.povRight().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ElevatorPosition.SCORE_L3));
    operator.povUp().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ElevatorPosition.SCORE_L4));
    // operator.rightBumper()
    // .and(() -> elevPos == ElevatorPosition.SCORE_L1 || elevPos ==
    // ElevatorPosition.SCORE_L2
    // || elevPos == ElevatorPosition.SCORE_L3 || elevPos ==
    // ElevatorPosition.SCORE_L4)
    // .onTrue(new ScoreCoral(coral));

    // operator.y().onTrue(new SetElevatorPosition(elevator, pivot,
    // ElevatorPosition.CLIMB));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }

  public void onEnable() {
    elevator.setSetpoint(elevator.getHeight());
    pivot.setSetpoint(pivot.getAngle());
  }

  private void changeMode(OperatorMode newMode) {
    mode = newMode;
  }

  public static void setShooterPosition(ShooterPosition newPos) {
    shootPos = newPos;
  }

  public static void setElevatorPosition(ElevatorPosition newPos) {
    elevPos = newPos;
  }
}
