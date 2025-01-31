// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootAlgae;
import frc.robot.commands.coral.IntakeCoral;
import frc.robot.commands.coral.ScoreCoralHigh;
import frc.robot.commands.coral.ScoreCoralLow;
import frc.robot.commands.coral.ScoreCoralMid;
import frc.robot.subsystems.AlgaeProcessor;
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
  private final AlgaeProcessor processor;
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
    processor = new AlgaeProcessor();
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
    driver.b().onTrue(Commands.none()); // need vision subsystem
    driver.y().onTrue(controls.toggleSlowmode());

    // OPERATOR
    operator.y().onTrue(new ShootAlgae(shooter));
    operator.povUp().onTrue(new ScoreCoralHigh(coral, pivot));
    operator.povLeft().onTrue(new ScoreCoralMid(coral, pivot));
    operator.povDown().onTrue(new ScoreCoralLow(coral, pivot));
    operator.povRight().onTrue(new IntakeCoral(coral, pivot));
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
