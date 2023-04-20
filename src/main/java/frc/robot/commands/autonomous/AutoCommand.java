package frc.robot.commands.autonomous;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.LowerArmCommand;
import frc.robot.commands.intake.ExtakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCommand extends SequentialCommandGroup {
  private boolean lowerArm = true;

  public AutoCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    addCommands(new ExtakeCommand(intakeSubsystem).withTimeout(3));
    addCommands(TaxiCommand.create(driveSubsystem));
    addCommands(new ConditionalCommand(new LowerArmCommand(armSubsystem), new InstantCommand(), () -> lowerArm));
  
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addBooleanProperty("lower arm", () -> lowerArm, val -> lowerArm = val);
  }

}
