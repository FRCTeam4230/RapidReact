// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.MotorID;

public class DriveSubsystem extends SubsystemBase {

  private final MotorControllerGroup MG1;
  private final MotorControllerGroup MG2;

  private Map<MotorID, CANSparkMax> motors = new HashMap<>();
  private Map<MotorID, RelativeEncoder> motorEncoders = new HashMap<>();

  private final DifferentialDrive differentialDrive;

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  private static Function<MotorID, CANSparkMax> createMotor = (id) -> {
    CANSparkMax motor = new CANSparkMax(id.getId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(DriveTrain.timeToFullSpeed);
    motor.setIdleMode(IdleMode.kCoast);

    RelativeEncoder maxEncoder = motor.getEncoder();
    maxEncoder.setPositionConversionFactor(DriveTrain.motorRotationsToInches);

    return motor;
  };

  public DriveSubsystem(List<MotorID> motorIds) {
    super();

    motorIds.forEach(motorId -> {

      CANSparkMax controller = createMotor.apply(motorId);
      motors.put(motorId, controller);
      motorEncoders.put(motorId, controller.getEncoder());

      switch (motorId) {
        case MG1_1:
        case MG1_2:

          break;
        case MG2_1:
        case MG2_2:
          controller.setInverted(true);
          break;
        //drive subsystem doesn't support any other motors
        default:
          break;
      }
    });

    MG1 = new MotorControllerGroup(motors.get(MotorID.MG1_1), motors.get(MotorID.MG1_2));
    MG2 = new MotorControllerGroup(motors.get(MotorID.MG2_1), motors.get(MotorID.MG2_2));

    differentialDrive = new DifferentialDrive(MG1, MG2);

    resetEncoders();
    setRampTime(DriveTrain.timeToFullSpeed);

    SmartDashboard.putData(this);

    navx.calibrate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double speed, double rotations, boolean squareInputs) {
    differentialDrive.arcadeDrive(speed, rotations, true);
  }

  public void arcadeDrive(double speed, double rotations) {
    differentialDrive.arcadeDrive(speed, rotations);
  }

  public void setSpeeds(double mg1Speed, double mg2Speed) {
    differentialDrive.tankDrive(mg1Speed, mg2Speed, false);
  }

  public double getAverageEncoderDistance() {
    return (getMg1Position() + getMg2Position()) / 2.0;
  }

  public void resetEncoders() {
    motors.entrySet().forEach(encoder -> encoder.getValue().getEncoder().setPosition(0));
  }

  public double getMg1Position() {
    return (motorEncoders.get(MotorID.MG1_1).getPosition() + motorEncoders.get(MotorID.MG1_2).getPosition()) / 2.0;
  }

  public double getMg2Position() {
    return (motorEncoders.get(MotorID.MG2_1).getPosition() + motorEncoders.get(MotorID.MG2_2).getPosition()) / 2.0;
  }

  public void setRampTime(double time) {
    motors.entrySet().forEach(motor -> {
      motor.getValue().setOpenLoopRampRate(time);
    });
  }

  public void stop() {
    differentialDrive.tankDrive(0, 0);
  }

  public double getRotation() {
    return navx.getAngle();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("MG1_Encoder", this::getMg1Position, null);
    builder.addDoubleProperty("MG2_Encoder", this::getMg2Position, null);

    builder.addDoubleProperty("ramp rate", () -> motors.get(MotorID.MG1_1).getOpenLoopRampRate(), null);

    builder.addDoubleProperty("angle", this::getRotation, null);
  }
}
