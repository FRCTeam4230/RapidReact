// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

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
  private final CANSparkMax[] mg1Motors;
  private final MotorControllerGroup MG1;

  private final CANSparkMax[] mg2Motors;
  private final MotorControllerGroup MG2;

  private final CANSparkMax[] motors;

  private final DifferentialDrive differentialDrive;

  private final RelativeEncoder MG1_Encoder1;
  private final RelativeEncoder MG1_Encoder2;
  private final RelativeEncoder MG2_Encoder1;
  private final RelativeEncoder MG2_Encoder2;

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  public DriveSubsystem(int[] mg1IDs, int[] mg2IDs) {
    super();

    motors = new CANSparkMax[mg1IDs.length + mg2IDs.length];

    mg1Motors = new CANSparkMax[mg1IDs.length];
    for (int i = 0; i < mg1IDs.length; i++) {
      mg1Motors[i] = new CANSparkMax(mg1IDs[i], MotorType.kBrushless);
      motors[i] = mg1Motors[i];
    }
    MG1 = new MotorControllerGroup(mg1Motors);

    mg2Motors = new CANSparkMax[mg2IDs.length];
    for (int i = 0; i < mg2IDs.length; i++) {
      mg2Motors[i] = new CANSparkMax(mg2IDs[i], MotorType.kBrushless);
      motors[i + mg1IDs.length] = mg2Motors[i];
    }
    MG2 = new MotorControllerGroup(mg2Motors);

    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
      motor.setOpenLoopRampRate(DriveTrain.timeToFullSpeed);
      motor.setIdleMode(IdleMode.kCoast);
    }

    MG1_Encoder1 = mg1Motors[0].getEncoder();
    MG1_Encoder2 = mg1Motors[1].getEncoder();
    MG2_Encoder1 = mg2Motors[0].getEncoder();
    MG2_Encoder2 = mg2Motors[1].getEncoder();

    Stream.of(MG1_Encoder1, MG1_Encoder2, MG2_Encoder1, MG2_Encoder2)
        .forEach(encoder -> encoder.setPositionConversionFactor(DriveTrain.motorRotationsToInches));

    Arrays.stream(mg2Motors).forEach(motor -> motor.setInverted(true));

    differentialDrive = new DifferentialDrive(MG1, MG2);

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
    return (getMg1Position() + getMg2Position()) / 2.0;
  }

  public void resetEncoders() {
    Stream.of(MG1_Encoder1, MG1_Encoder2, MG2_Encoder1, MG2_Encoder2)
        .forEach(encoder -> encoder.setPosition(0));
  }

  public double getMg1Position() {
    return (MG1_Encoder1.getPosition() + MG1_Encoder2.getPosition()) / 2;
  }

  public double getMg2Position() {
    return (MG2_Encoder1.getPosition() + MG2_Encoder2.getPosition()) / 2;
  }

  public void setRampTime(double time) {
    for (var motor : motors) {
      motor.setOpenLoopRampRate(time);
    }
  }

  public void stop() {
    differentialDrive.tankDrive(0, 0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("MG1_Encoder", this::getMg1Position, null);
    builder.addDoubleProperty("MG2_Encoder", this::getMg2Position, null);
  }
}
