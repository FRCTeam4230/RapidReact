package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;

  DoubleSupplier xSupplier, ySupplier;
  BooleanSupplier speedToggledSupplier;

  private boolean speedToggled;

  public TeleopCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSupplier, 
  DoubleSupplier ySupplier, BooleanSupplier speedToggledSupplier) {
    this.driveSubsystem = driveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.speedToggledSupplier = speedToggledSupplier;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    speedToggled = false;
  }

  @Override
  public void execute() {
    //Change speed toggled if the button is pressed
    if (speedToggledSupplier.getAsBoolean()) {
      speedToggled = !speedToggled;
    }

    driveSubsystem.arcadeDrive(
      ySupplier.getAsDouble() * (!speedToggled ? DriveTrain.moveMult : DriveTrain.moveMult2), 
      xSupplier.getAsDouble() * (!speedToggled ? DriveTrain.turnMult : DriveTrain.turnMult2));
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {}
}
