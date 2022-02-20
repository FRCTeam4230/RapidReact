// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private final XboxController controller;

  public static double moveMult = DriveTrain.moveMult;
  public static double turnMult = DriveTrain.turnMult;

  /** Creates a new TeleopCommand. */
  public TeleopCommand(DriveSubsystem driveSubsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;

    addRequirements(driveSubsystem);

    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setRampTime(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(controller.getLeftY() * moveMult,
        controller.getRightX() * turnMult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setRampTime(DriveTrain.timeToFullSpeed);
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("move speed", () -> moveMult, s -> moveMult = s);
    builder.addDoubleProperty("turn speed", () -> turnMult, s -> turnMult = s);
  }
}
