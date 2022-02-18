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

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax heightMotor = new CANSparkMax(5, MotorType.kBrushless);
  private final RelativeEncoder heightEncoder = heightMotor.getEncoder();
  private final DigitalInput upperLimitSwitch = new DigitalInput(0);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(1);

  public enum State {
    MOVING_UP,
    MOVING_DOWN,
    UP,
    DOWN;
  }

  private State state = State.DOWN;

  /** Creates a new InakteSubsystem. */
  public IntakeSubsystem() {
    super();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (state == State.MOVING_UP && upperLimitSwitch.get()) {
      state = State.UP;
      heightMotor.stopMotor();
    } else if (state == State.MOVING_DOWN && lowerLimitSwitch.get()) {
      state = State.DOWN;
      heightMotor.stopMotor();
    }
  }

  public void lower() {
    state = State.MOVING_DOWN;
    heightMotor.set(-0.1);
  }

  public void raise() {
    state = State.MOVING_UP;
    heightMotor.set(0.1);
  }

  public void extake() {
  }

  public void intake() {
  }

  public double getPosition() {
    return heightEncoder.getPosition();
  }

  public State getState() {
    return state;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("encoder postion", this::getPosition, null);
  }
}
