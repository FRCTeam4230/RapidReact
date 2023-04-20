package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Climber;
import frc.robot.Constants.DigitalIOIDs;
import frc.robot.Constants.MotorID;
import frc.robot.commands.ManualClimbCommand;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.arm.LowerArmCommand;
import frc.robot.commands.arm.PIDHoldArm;
import frc.robot.commands.arm.RaiseArmCommand;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.intake.ManualIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClimberSubsystem leftClimberSubsystem = new ClimberSubsystem(MotorID.LEFT_CLIMBER.getId(),
      DigitalIOIDs.leftClimber, Climber.leftSpeedMult);
  private final ClimberSubsystem rightClimberSubsystem = new ClimberSubsystem(MotorID.RIGHT_CLIMBER.getId(),
      DigitalIOIDs.rightClimber, Climber.rightSpeedMult);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Controllers
  private final XboxController driverController = new XboxController(0);
  private final XboxController secondController = new XboxController(1);

  // Commands
  private final DoubleSupplier intakeSupplier = 
  () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis();
  private final ManualIntakeCommand intakeCommand = new ManualIntakeCommand(intakeSubsystem, intakeSupplier);
  //Button A to speed toggle
  private final TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, 
  () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getAButton());
  private final ManualClimbCommand lClimbCommand = new ManualClimbCommand(leftClimberSubsystem,
      secondController::getLeftY);
  private final ManualClimbCommand rClimbCommand = new ManualClimbCommand(rightClimberSubsystem,
      secondController::getRightY);
  private final Command resetClimbersCommand = new ResetClimber(leftClimberSubsystem)
      .alongWith(new ResetClimber(rightClimberSubsystem));

  //Auto command
  private final Command autoCommand = new AutoCommand(driveSubsystem, intakeSubsystem, armSubsystem);


  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
    CameraServer.startAutomaticCapture();
  }

  public Command getTeleopCommand() {
    return resetClimbersCommand;
  }

  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new LowerArmCommand(armSubsystem));

    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RaiseArmCommand(armSubsystem));
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(teleopCommand);

    leftClimberSubsystem.setDefaultCommand(lClimbCommand);
    rightClimberSubsystem.setDefaultCommand(rClimbCommand);

    intakeSubsystem.setDefaultCommand(intakeCommand);

    armSubsystem.setDefaultCommand(new PIDHoldArm(armSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
