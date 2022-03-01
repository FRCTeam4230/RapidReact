// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DigitalIOIDs;
import frc.robot.Constants.Intake;
import frc.robot.Constants.MotorID;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(MotorID.ARM.getId(), MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final DigitalInput lowerLimitSwitch = new DigitalInput(DigitalIOIDs.lowerArmLimit);
  private final DigitalInput upperLimitSwitch = new DigitalInput(DigitalIOIDs.upperArmLimit);

  /** Creates a new InakteSubsystem. */
  public ArmSubsystem() {
    super();

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setOpenLoopRampRate(Constants.motorRampTime);

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((isUp() && motor.get() > Intake.upHoldSpeed) || (isDown() && motor.get() < 0)) {
      stop();
    }
  }

  public void stop() {
    setSpeed(0);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public boolean isUp() {
    return !upperLimitSwitch.get();
  }

  public boolean isDown() {
    return !lowerLimitSwitch.get();
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("encoder postion", this::getPosition, encoder::setPosition);
    builder.addBooleanProperty("lower limit", lowerLimitSwitch::get, null);
    builder.addBooleanProperty("upper limit", upperLimitSwitch::get, null);
    builder.addDoubleProperty("hold speed", () -> Intake.holdSpeed, n -> Intake.holdSpeed = n);
    builder.addDoubleProperty("upper hold speed", () -> Intake.upHoldSpeed, n -> Intake.upHoldSpeed = n);
  }
}
