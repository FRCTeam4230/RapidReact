// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
  private final DigitalInput limitSwitch;

  private final double speedMult;

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(int SparkMaxCANID, int limitSwitchID, double speedMult) {
    super();

    limitSwitch = new DigitalInput(limitSwitchID);

    motor = new CANSparkMax(SparkMaxCANID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.MOTOR_RAMP_TIME);
    useBrake(true);

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    SmartDashboard.putData("climber on motor " + motor.getDeviceId(), this);

    this.speedMult = speedMult;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!canUseSpeed(motor.get())) {
      stop();
    }
  }

  public void stop() {
    setSpeed(0);
  }

  public void toggleInvert() {
    motor.setInverted(!motor.getInverted());
  }

  private void useBrake(boolean brake) {
    motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public boolean atLimit() {
    return !limitSwitch.get();
  }

  private boolean canUseSpeed(double speed) {
    if (limitSwitch.get() || speed == 0) {
      return true; // no issues because not triggering limit switch or trying to stop
    }

    //What does this line mean?
    return Math.signum(speed) != Math.signum(encoder.getPosition() - Climber.highLimit);
  }

  public void setSpeed(double speed) {
    if (!canUseSpeed(speed)) {
      return;
    }
    motor.set(MathUtil.clamp(speed, -1, 1) * speedMult);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("encoder postion", this::getPosition, null);
    builder.addDoubleProperty("encoder postion - high limit", () -> encoder.getPosition() - Climber.highLimit, null);
    builder.addDoubleProperty("encoder speed", encoder::getVelocity, null);
    builder.addBooleanProperty("limit switch", limitSwitch::get, null);

    builder.addDoubleProperty("motor speed", motor::get, motor::set);
  }
}
