// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final XboxController controller = new XboxController(0);

  private final TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, controller);

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
    joystickButton.whenPressed(new DriveDistance(driveSubsystem));

    makeButton(XboxController.Button.kA, new InstantCommand(driveSubsystem::resetEncoders, driveSubsystem));

    makeButton(XboxController.Button.kX, new InstantCommand(intakeSubsystem::raise, intakeSubsystem));
    makeButton(XboxController.Button.kY, new InstantCommand(intakeSubsystem::lower, intakeSubsystem));

    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, teleopCommand);
  }

  private void makeButton(XboxController.Button button, Command command) {
    new JoystickButton(controller, button.value).whenPressed(command);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
