// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    super();
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    MathUtil.clamp(speed, -1, 1);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("encoder postion", this::getPosition, null);
    builder.addDoubleProperty("encoder speed", encoder::getVelocity, null);
  }
}
