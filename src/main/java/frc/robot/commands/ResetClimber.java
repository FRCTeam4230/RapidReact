// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetClimber extends SequentialCommandGroup {
  /** Creates a new ResetClimbersFull. */
  public ResetClimber(ClimberSubsystem climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(climber::resetEncoder, climber),

        new WaitUntilCommand(() -> Math.abs(climber.getPosition()) > Climber.resetRotations)
            .raceWith(new MoveClimbersToBottom(climber)),

        new ConditionalCommand(
            new InstantCommand(),
            new InstantCommand(climber::toggleInvert, climber).andThen(new MoveClimbersToBottom(climber)),
            climber::atLimit),

        new InstantCommand(climber::resetEncoder, climber));

    SmartDashboard.putData(this);
  }
}
