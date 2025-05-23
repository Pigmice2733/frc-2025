// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.LEDConfig;
import frc.robot.Constants.OperatorMode;
import frc.robot.Constants.ShooterConfig;
import frc.robot.commands.AlgaeAuto;
import frc.robot.commands.CenterAlgae;
import frc.robot.commands.CoralAuto;
import frc.robot.commands.DriveAndTurnFieldRelative;
import frc.robot.commands.DriveJoysticks;
import frc.robot.commands.DrivePath;
import frc.robot.commands.ElevatorControl;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.PivotControl;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.ShootNet;
import frc.robot.commands.ShootProcessor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
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
  private final Grabber grabber;
  private final Shooter shooter;
  private final Elevator elevator;
  private final Pivot pivot;
  private final Vision vision;
  private final Underglow underglow;

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private final Controls controls;

  private OperatorMode mode;
  public static ArmPosition elevPos;
  private boolean robotOriented, slowSave;

  private SendableChooser<Command> autoChooser;
  private PathPlannerPath sampleAutoPath;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);
    controls = new Controls(driver, operator);

    drivetrain = new Drivetrain();
    grabber = new Grabber();
    shooter = new Shooter();
    elevator = new Elevator();
    pivot = new Pivot();
    vision = new Vision();
    underglow = new Underglow();
    mode = OperatorMode.NONE;
    elevPos = ArmPosition.STOW;

    robotOriented = false;
    slowSave = false;

    autoChooser = new SendableChooser<Command>();

    underglow.setSolidColor(LEDConfig.PURPLE);

    SmartDashboard.putString("Elevator Position", "stow");
    SmartDashboard.putString("Operator Mode", "");

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
        controls::getTurnSpeed,
        () -> robotOriented));

    elevator.setDefaultCommand(new ElevatorControl(elevator, controls::getElevatorSpeed));
    pivot.setDefaultCommand(new PivotControl(pivot, controls::getPivotSpeed));
    // shooter.setDefaultCommand(shooter.changePivotSetpoint(controls::getShooterSpeed));
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
    driver.rightBumper().onTrue(new InstantCommand(() -> {
      slowSave = controls.getSlowmode();
      controls.setSlowmode(true);
      robotOriented = true;
    })).onFalse(new InstantCommand(() -> {
      controls.setSlowmode(slowSave);
      robotOriented = false;
    }));
    driver.povDown()
        .whileTrue(getDriveVisionCommand(drivetrain, vision, driver, Units.inchesToMeters(
            16.5), 0, 0))
        .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0)));
    driver.povRight()
        .whileTrue(getDriveVisionCommand(drivetrain, vision, driver, Units.inchesToMeters(
            16.5),
            Units.inchesToMeters(8), 0))
        .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0)));
    driver.povLeft()
        .whileTrue(getDriveVisionCommand(drivetrain, vision, driver, Units.inchesToMeters(
            16.5),
            -Units.inchesToMeters(4), 0))
        .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0)));

    // OPERATOR
    operator.b().onTrue(new InstantCommand(() -> changeMode(OperatorMode.ELEVATOR)));
    operator.povDown().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.STOW));
    operator.povLeft().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.HUMAN_PLAYER));
    operator.povRight().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.ALGAE_L2))
        .onTrue(Commands
            .runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_REEF_INTAKE_ANGLE)));
    operator.povUp().and(() -> mode == OperatorMode.ELEVATOR)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.ALGAE_L3))
        .onTrue(Commands
            .runOnce(() -> shooter.setPivotPositionSetpoint(ShooterConfig.PIVOT_REEF_INTAKE_ANGLE)));

    operator.x().onTrue(new InstantCommand(() -> changeMode(OperatorMode.REEF)));
    operator.povDown().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L1))
        .onTrue(Commands.runOnce(() -> shooter.stowPivotIfNoAlgae()));

    operator.povLeft().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L2))
        .onTrue(Commands.runOnce(() -> shooter.stowPivotIfNoAlgae()));
    operator.povRight().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L3))
        .onTrue(Commands.runOnce(() -> shooter.stowPivotIfNoAlgae()));
    operator.povUp().and(() -> mode == OperatorMode.REEF)
        .onTrue(new SetArmPosition(elevator, pivot, ArmPosition.SCORE_L4))
        .onTrue(Commands.runOnce(() -> shooter.stowPivotIfNoAlgae()));

    operator.leftBumper().and(() -> mode == OperatorMode.ELEVATOR || mode == OperatorMode.REEF)
        .onTrue(grabber.runForward()).onFalse(grabber.stopMotor());
    operator.rightBumper()
        .and(() -> mode == OperatorMode.REEF || mode == OperatorMode.ELEVATOR)
        .onTrue(grabber.runReverse()).onFalse(grabber.stopMotor());

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
        .onTrue(new IntakeAlgae(shooter))
        .onFalse(shooter.stopMotors());
    operator.leftBumper().and(() -> mode == OperatorMode.ELEVATOR)
        .and(() -> elevPos == ArmPosition.ALGAE_L2 || elevPos == ArmPosition.ALGAE_L3)
        .onTrue(new IntakeAlgae(shooter)).onFalse(shooter.stopMotors())
        .onFalse(Commands.runOnce(() -> shooter.stowPivotIfNoAlgae()));

    operator.rightBumper().and(() -> mode == OperatorMode.SHOOTER).and(operator.leftTrigger().negate())
        .onTrue(new ShootProcessor(shooter)).onFalse(shooter.stopMotors());
    operator.leftTrigger().and(() -> mode == OperatorMode.SHOOTER)
        .onTrue(new PrepareToShoot(shooter, operator))
        .onFalse(shooter.stopMotors())
        .onFalse(Commands.runOnce(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
    operator.rightBumper().and(() -> mode == OperatorMode.SHOOTER).and(operator.leftTrigger())
        .onTrue(new ShootNet(shooter, operator));
    operator.rightTrigger().and(() -> mode == OperatorMode.SHOOTER).onTrue(new CenterAlgae(shooter));

    // operator.a().whileTrue(pivot.sysIdDynamic(Direction.kForward));
    // operator.b().whileTrue(pivot.sysIdDynamic(Direction.k
    // operator.x().whileTrue(pivot.sysIdQuasistatic(Direction.kForward));
    // operator.y().whileTrue(pivot.sysIdQuasistatic(Direction.kReverse));
  }

  public static Command getDriveVisionCommand(Drivetrain drivetrain, Vision vision, CommandXboxController controller,
      double xOffset,
      double yOffset, double rOffset) {
    return new DriveAndTurnFieldRelative(drivetrain, vision, controller, xOffset, yOffset, rOffset);
  }

  /**
   * Puts the autonomous-command options in Elastic.
   */
  private void buildAutoChooser() {
    autoChooser.addOption("None", Commands.none());
    autoChooser.addOption("Drive Only", new DrivePath(drivetrain, new Transform2d(-2, 0, new Rotation2d())));
    autoChooser.addOption("Score L4", new CoralAuto(drivetrain, vision, elevator, pivot, grabber, driver));
    autoChooser.addOption("Score & L2 Algae",
        new AlgaeAuto(drivetrain, vision, elevator, pivot, grabber, shooter, driver, false));
    autoChooser.addOption("Score & L3 Algae",
        new AlgaeAuto(drivetrain, vision, elevator, pivot, grabber, shooter, driver, true));

    try {
      sampleAutoPath = PathPlannerPath.fromPathFile("sample auto");
    } catch (Exception e) {
      sampleAutoPath = null;
    }
    autoChooser.addOption("Sample Path",
        AutoBuilder.pathfindThenFollowPath(sampleAutoPath, drivetrain.getConstraints()));

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
    grabber.setSpeed(0, true);
    shooter.setPivotPositionSetpoint(0);
    controls.setSlowmode(false);
    changeMode(OperatorMode.NONE);
  }

  public void autoInit() {
    drivetrain.resetPose(new Pose2d());
  }

  private void changeMode(OperatorMode newMode) {
    mode = newMode;
    SmartDashboard.putString("Operator Mode", mode.name().toLowerCase());
  }

  public static void setArmPosition(ArmPosition newPos) {
    elevPos = newPos;
    SmartDashboard.putString("Elevator Position", newPos.toString().toLowerCase());
  }
}
