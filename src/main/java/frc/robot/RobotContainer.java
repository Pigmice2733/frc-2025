// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorPosition;
import frc.robot.commands.AlgaeFromReef;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ShootProcessor;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.ShootAlgae;
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
  private final AlgaeGrabber processor;
  private final AlgaeShooter shooter;
  private final CoralManipulator coral;
  private final Elevator elevator;
  private final Pivot pivot;

  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private final Controls controls;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);
    controls = new Controls(driver);

    drivetrain = new Drivetrain();
    processor = new AlgaeGrabber();
    shooter = new AlgaeShooter();
    coral = new CoralManipulator();
    elevator = new Elevator();
    pivot = new Pivot();

    // Configure the trigger bindings
    configureBindings();
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

    // OPERATOR TODO
    operator.rightBumper().onTrue(new ScoreCoral(coral));
    operator.leftBumper().onTrue(new IntakeCoral(coral));
    operator.povUp().onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L4));
    operator.povLeft().onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L3));
    operator.povRight().onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L2));
    operator.povDown().onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.SCORE_L1));
    operator.x().onTrue(new SetElevatorPosition(elevator, pivot, ElevatorPosition.HUMAN_PLAYER));
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
}
