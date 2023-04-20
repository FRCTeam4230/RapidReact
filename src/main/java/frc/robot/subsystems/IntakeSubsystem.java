// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorID;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSPX motor = new VictorSPX(MotorID.INTAKE.getId());

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    super();

    motor.configFactoryDefault();
    motor.configOpenloopRamp(Constants.MOTOR_RAMP_TIME);

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    setSpeed(0);
  }

  public void setSpeed(double speed) {
    motor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("motor speed", motor::getMotorOutputPercent, this::setSpeed);
  }
}
