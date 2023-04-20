package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class RaiseArmCommand extends AbstractArmCommand {
  public RaiseArmCommand(ArmSubsystem armSubsystem) {
    super(armSubsystem, Direction.UP);
  }
}
