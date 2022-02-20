// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DigitalIOIDs;
import frc.robot.Constants.MotorID;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualClimbCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.autonomous.TaxiCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSubsystem = new DriveSubsystem(
      Arrays.asList(MotorID.MG1_1, MotorID.MG1_2, MotorID.MG2_1, MotorID.MG2_2));
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClimberSubsystem leftClimberSubsystem = new ClimberSubsystem(MotorID.LEFT_CLIMBER.getId(),
      DigitalIOIDs.leftClimber);
  private final ClimberSubsystem rightClimberSubsystem = new ClimberSubsystem(MotorID.RIGHT_CLIMBER.getId(),
      DigitalIOIDs.rightClimber);
  private final XboxController controller = new XboxController(0);
  private final XboxController secondController = new XboxController(1);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DoubleSupplier intakeSupplier = () -> controller.getLeftTriggerAxis()
      - controller.getRightTriggerAxis();
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, intakeSupplier);

  private final TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, controller);

  private final ManualClimbCommand lClimbCommand = new ManualClimbCommand(leftClimberSubsystem,
      secondController::getLeftY);
  private final ManualClimbCommand rClimbCommand = new ManualClimbCommand(rightClimberSubsystem,
      secondController::getRightY);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public Command getTeleopCommand() {
    return teleopCommand;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private final JoystickButton joystickButton = new JoystickButton(controller, XboxController.Button.kX.value);

  private void configureButtonBindings() {
    joystickButton.whenPressed(DriveDistance.create(driveSubsystem));

    makeButton(XboxController.Button.kA, new InstantCommand(driveSubsystem::resetEncoders, driveSubsystem));

    makeButton(XboxController.Button.kB, new InstantCommand(armSubsystem::raise, armSubsystem));
    makeButton(XboxController.Button.kY, new InstantCommand(armSubsystem::lower, armSubsystem));

    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, teleopCommand);

    CommandScheduler.getInstance().setDefaultCommand(leftClimberSubsystem, lClimbCommand);
    CommandScheduler.getInstance().setDefaultCommand(rightClimberSubsystem, rClimbCommand);

    CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, intakeCommand);
  }

  private void makeButton(XboxController.Button button, Command command) {
    new JoystickButton(controller, button.value).whenPressed(command);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private final Command autoCommand = createAutoComamand();

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return autoCommand;
  }

  private Command createAutoComamand() {
    return TaxiCommand.create(driveSubsystem);
  }
}
