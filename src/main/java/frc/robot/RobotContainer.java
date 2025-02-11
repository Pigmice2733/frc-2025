// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetShooterPosition;
import frc.robot.commands.ShootNet;
import frc.robot.commands.ShootProcessor;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.Climber;
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
  private final AlgaeShooter shooter;
  private final Climber climber;
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
    shooter = new AlgaeShooter();
    climber = new Climber();
    coral = new CoralManipulator();
    elevator = new Elevator();
    pivot = new Pivot();

    mode = OperatorMode.NONE;
    elevPos = ElevatorPosition.STOW;
    shootPos = ShooterPosition.STOW;

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DriveJoysticks(
        drivetrain,
        controls::getDriveSpeedX,
        controls::getDriveSpeedY,
        controls::getTurnSpeed));
    elevator.setDefaultCommand(elevator.manualSpeed(controls.getElevatorSpeed()));
    pivot.setDefaultCommand(pivot.manualSpeed(controls.getPivotSpeed()));
    shooter.setDefaultCommand(shooter.manualSpeed(controls.getShooterSpeed()));
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
    // create vision commands once Vision subsystem exists

    // OPERATOR
    operator.a().onTrue(new InstantCommand(() -> changeMode(OperatorMode.SHOOTER)));
    operator.povDown().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new SetShooterPosition(shooter, ShooterPosition.INTAKE));
    operator.povRight().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new SetShooterPosition(shooter, ShooterPosition.PROCESSOR));
    operator.povUp().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new SetShooterPosition(shooter, ShooterPosition.NET));
    operator.povLeft().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new SetShooterPosition(shooter, ShooterPosition.STOW));
    operator.leftBumper().and(() -> (shootPos == ShooterPosition.INTAKE))
        .onTrue(new IntakeAlgae(shooter));
    operator.leftBumper().and(() -> (shootPos == ShooterPosition.NET)).onTrue(new ShootNet(shooter));
    operator.leftBumper().and(() -> (shootPos == ShooterPosition.PROCESSOR))
        .onTrue(new ShootProcessor(shooter));

    operator.b().onTrue(new InstantCommand(() -> changeMode(OperatorMode.ELEVATOR)));
    operator.povDown().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.STOW));
    operator.povLeft().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.HUMAN_PLAYER));
    operator.povRight().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.REEF_L2));
    operator.povUp().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.REEF_L3));
    operator.rightBumper().and(() -> (elevPos == ElevatorPosition.HUMAN_PLAYER))
        .onTrue(new IntakeCoral(coral));
    operator.rightBumper().and(() -> (elevPos == ElevatorPosition.REEF_L2
        || elevPos == ElevatorPosition.REEF_L3)).onTrue(new AlgaeFromReef(grabber));

    operator.x().onTrue(new InstantCommand(() -> changeMode(OperatorMode.CORAL)));
    operator.povDown().and(() -> mode == OperatorMode.CORAL)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L1));
    operator.povLeft().and(() -> mode == OperatorMode.CORAL)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L2));
    operator.povRight().and(() -> mode == OperatorMode.CORAL)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L3));
    operator.povUp().and(() -> mode == OperatorMode.CORAL)
        .onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L4));
    operator.rightBumper()
        .and(() -> elevPos == ElevatorPosition.SCORE_L1 || elevPos == ElevatorPosition.SCORE_L2
            || elevPos == ElevatorPosition.SCORE_L3 || elevPos == ElevatorPosition.SCORE_L4)
        .onTrue(new ScoreCoral(coral));

    operator.y().onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.CLIMB))
        .onTrue(climber.climbPosition());
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
