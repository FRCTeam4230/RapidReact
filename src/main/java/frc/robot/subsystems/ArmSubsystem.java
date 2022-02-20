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

  public enum State {
    MOVING_UP,
    MOVING_DOWN,
    UP,
    DOWN;
  }

  private State state = !lowerLimitSwitch.get() ? State.DOWN : State.UP;

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
    if (state == State.MOVING_UP && !upperLimitSwitch.get()) {
      state = State.UP;
      motor.stopMotor();
    } else if (state == State.MOVING_DOWN && !lowerLimitSwitch.get()) {
      state = State.DOWN;
      motor.stopMotor();
    }
  }

  public void lower() {
    if (state == State.DOWN)
      return;
    state = State.MOVING_DOWN;
    motor.set(Intake.downArmSpeed);
  }

  public void raise() {
    if (state == State.UP)
      return;
    state = State.MOVING_UP;
    motor.set(Intake.upArmSpeed);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public State getState() {
    return state;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("encoder postion", this::getPosition, encoder::setPosition);
    builder.addStringProperty("state", () -> getState().toString(), null);
    builder.addBooleanProperty("lower limit", lowerLimitSwitch::get, null);
    builder.addBooleanProperty("upper limit", upperLimitSwitch::get, null);
  }
}
