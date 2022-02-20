// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurnCommandParams;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnCommand extends PIDCommand {
  private final DriveSubsystem driveSubsystem;

  private static final DoubleSupplier getMeasurementSupplier(DoubleSupplier rotationSupplier) {
    return () -> (rotationSupplier.getAsDouble() + 180) % 360 - 180;
  }

  public TurnCommand(DriveSubsystem driveSubsystem, double angle) {
    super(
        // The controller that the command will use
        new PIDController(TurnCommandParams.kP, TurnCommandParams.kI, TurnCommandParams.kD),
        getMeasurementSupplier(driveSubsystem::getRotation),
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          output += Math.copySign(0, output);
          driveSubsystem.arcadeDrive(0, output, false);
        });
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(TurnCommandParams.tolerance, TurnCommandParams.velocityTolerance);

    SmartDashboard.putData(this);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("kP", m_controller::getP, m_controller::setP);
    builder.addDoubleProperty("kI", m_controller::getI, m_controller::setI);
    builder.addDoubleProperty("kD", m_controller::getD, m_controller::setD);
    builder.addDoubleProperty("error", m_controller::getPositionError, null);
  }
}
