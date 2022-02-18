// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(7, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final DigitalInput lowerLimitSwitch = new DigitalInput(9);

  public enum State {
    MOVING_UP,
    MOVING_DOWN,
    UP,
    DOWN;
  }

  private State state = State.DOWN;

  /** Creates a new InakteSubsystem. */
  public ArmSubsystem() {
    super();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (state == State.MOVING_UP && encoder.getPosition() > Intake.Limits.up) {
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
    motor.set(-Intake.armSpeed);
  }

  public void raise() {
    if (state == State.UP)
      return;
    state = State.MOVING_UP;
    motor.set(Intake.armSpeed);
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
  }
}
