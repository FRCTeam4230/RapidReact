package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class PIDHoldArm extends CommandBase {
  ArmSubsystem armSubsystem;
  PIDController pidController;
  public PIDHoldArm(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    pidController = new PIDController(Constants.ArmConstants.kP, 0, 0);

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(armSubsystem.getPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = pidController.calculate(armSubsystem.getPosition());
    armSubsystem.setSpeed(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
