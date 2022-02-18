// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax MG1_SPARK_1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax MG1_SPARK_2 = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorControllerGroup MG1 = new MotorControllerGroup(MG1_SPARK_1, MG1_SPARK_2);

  private final CANSparkMax MG2_SPARK_3 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax MG2_SPARK_4 = new CANSparkMax(4, MotorType.kBrushless);
  private final MotorControllerGroup MG2 = new MotorControllerGroup(MG2_SPARK_3, MG2_SPARK_4);

  private final CANSparkMax[] motors = { MG1_SPARK_1, MG1_SPARK_2, MG2_SPARK_3, MG2_SPARK_4 };

  private final DifferentialDrive differentialDrive = new DifferentialDrive(MG1, MG2);

  private final RelativeEncoder MG1_Encoder = MG1_SPARK_1.getEncoder();
  private final RelativeEncoder MG2_Encoder = MG2_SPARK_3.getEncoder();

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  public DriveSubsystem() {
    super();
    MG2_SPARK_3.setInverted(true);
    MG2_SPARK_4.setInverted(true);

    MG1_Encoder.setPositionConversionFactor(DriveTrain.motorRotationsToInches);
    MG2_Encoder.setPositionConversionFactor(DriveTrain.motorRotationsToInches);

    for (CANSparkMax motor : motors) {
      motor.setOpenLoopRampRate(3);
      motor.setIdleMode(IdleMode.kCoast);
    }

    resetEncoders();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double speed, double rotations) {
    differentialDrive.arcadeDrive(speed, rotations);
  }

  public void setSpeeds(double mg1Speed, double mg2Speed) {
    differentialDrive.tankDrive(mg1Speed, mg2Speed, false);
  }

  public double getAverageEncoderDistance() {
    return (MG1_Encoder.getPosition() + MG2_Encoder.getPosition()) / 2.0;
  }

  public void resetEncoders() {
    MG1_Encoder.setPosition(0);
    MG2_Encoder.setPosition(0);
  }

  public void stop() {
    differentialDrive.tankDrive(0, 0);
  }

  public double getMg1Position() {
    return MG1_Encoder.getPosition();
  }

  public double getMg2Position() {
    return MG2_Encoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("MG1_Encoder", MG1_Encoder::getPosition, MG1_Encoder::setPosition);
    builder.addDoubleProperty("MG2_Encoder", MG2_Encoder::getPosition, MG2_Encoder::setPosition);
  }
}
