// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.*;
import frc.robot.commands.CoralAuto;
import frc.robot.commands.DriveJoysticks;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.ElevatorControl;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.PivotControl;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.ShootNet;
import frc.robot.commands.ShootProcessor;
import frc.robot.subsystems.GrabberWheel;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.Vision;

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
  private final GrabberWheel wheel;
  private final Shooter shooter;
  private final CoralGrabber coral;
  private final Elevator elevator;
  private final Pivot pivot;
  private final Vision vision;
  private final Underglow underglow;

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private final Controls controls;

  private OperatorMode mode;
  public static ArmPosition elevPos;

  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);
    controls = new Controls(driver, operator);

    drivetrain = new Drivetrain();
    wheel = new GrabberWheel();
    shooter = new Shooter();
    coral = new CoralGrabber();
    elevator = new Elevator();
    pivot = new Pivot();
    vision = new Vision();
    underglow = new Underglow();
    underglow.displayPigmicePurple();
    mode = OperatorMode.NONE;
    elevPos = ArmPosition.STOW;

    SmartDashboard.putString("Elevator Position", "stow");

    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
    buildAutoChooser();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Sets up commands to run for various subsystems when nothing else is
   * happening.
   */
  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DriveJoysticks(
        drivetrain,
        controls::getDriveSpeedX,
        controls::getDriveSpeedY,
        controls::getTurnSpeed));

    elevator.setDefaultCommand(new ElevatorControl(elevator, controls::getElevatorSpeed));
    pivot.setDefaultCommand(new PivotControl(pivot, controls::getPivotSpeed));
    shooter.setDefaultCommand(shooter.changePivotSetpoint(controls::getPivotSpeed));
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
    driver.povDown().whileTrue(new DriveToTarget(drivetrain, vision, Units.inchesToMeters(17), 0, 0));
    driver.povRight()
        .whileTrue(
            new DriveToTarget(drivetrain, vision, Units.inchesToMeters(17), -Units.inchesToMeters(6.5), 0));
    driver.povLeft()
        .whileTrue(
            new DriveToTarget(drivetrain, vision, Units.inchesToMeters(17), Units.inchesToMeters(6.5), 0));

    // OPERATOR
    operator.b().onTrue(new InstantCommand(() -> changeMode(OperatorMode.ELEVATOR)));
    operator.povDown().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot,
            ArmPosition.STOW));
    operator.povLeft().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot,
            ArmPosition.HUMAN_PLAYER));
    operator.povRight().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.ALGAE_L2));
    operator.povUp().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.ALGAE_L3));
    operator.rightBumper().and(() -> (elevPos == ArmPosition.HUMAN_PLAYER))
        .onTrue(new IntakeCoral(coral, wheel));
    operator.rightBumper().and(() -> (elevPos == ArmPosition.ALGAE_L2
        || elevPos == ArmPosition.ALGAE_L3)).onTrue(wheel.runForward()).onFalse(wheel.stopMotor());

    operator.x().onTrue(new InstantCommand(() -> changeMode(OperatorMode.REEF)));
    operator.povDown().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L1));
    operator.povLeft().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L2));
    operator.povRight().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L3));
    operator.povUp().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L4));
    operator.rightBumper()
        .and(() -> elevPos == ArmPosition.SCORE_L1 || elevPos == ArmPosition.SCORE_L2
            || elevPos == ArmPosition.SCORE_L3 || elevPos == ArmPosition.SCORE_L4)
        .onTrue(coral.outtake()).onTrue(wheel.runForward()).onFalse(coral.stopMotor())
        .onFalse(wheel.stopMotor());

    operator.y().whileTrue(new SetArmPosition(elevator, pivot, ArmPosition.CLIMB));

    operator.a().onTrue(new InstantCommand(() -> changeMode(OperatorMode.SHOOTER)));
    operator.povDown().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(Commands.runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_INTAKE_ANGLE)));
    operator.povRight().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(Commands.runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_NET_ANGLE)));
    operator.povLeft().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(Commands.runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_PROCESSOR_ANGLE)));
    operator.povUp().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(Commands.runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_STOW_ANGLE)));
    operator.leftBumper().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new IntakeAlgae(shooter)).onFalse(shooter.stopMotors());
    operator.rightBumper().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new ShootProcessor(shooter)).onFalse(shooter.stopMotors());
    operator.leftTrigger().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new PrepareToShoot(shooter, operator))
        .onFalse(shooter.stopMotors());
    operator.rightBumper().and(() -> mode == OperatorMode.SHOOTER).and(operator.leftTrigger())
        .onTrue(new ShootNet(shooter));

    // operator.a().whileTrue(pivot.sysIdDynamic(Direction.kForward));
    // operator.b().whileTrue(pivot.sysIdDynamic(Direction.k
    // operator.x().whileTrue(pivot.sysIdQuasistatic(Direction.kForward));
    // operator.y().whileTrue(pivot.sysIdQuasistatic(Direction.kReverse));
  }

  /**
   * Puts the autonomous-command options in Elastic.
   */
  private void buildAutoChooser() {
    autoChooser.addOption("None", Commands.none());
    autoChooser.addOption("Drive Only", drivetrain.simpleAuto());
    autoChooser.addOption("Score L3", new CoralAuto(drivetrain, vision, elevator, pivot, wheel, coral));

    SmartDashboard.putData("Autonomous Command", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void onEnable() {
    elevator.setSetpoint(elevator.getHeight());
    pivot.setSetpoint(pivot.getAngle());
    wheel.setSpeed(0);
    coral.setSpeed(0);
  }

  private void changeMode(OperatorMode newMode) {
    mode = newMode;
  }

  public static void setArmPosition(ArmPosition newPos) {
    elevPos = newPos;
    SmartDashboard.putString("Elevator Position", newPos.toString().toLowerCase());
  }
}
