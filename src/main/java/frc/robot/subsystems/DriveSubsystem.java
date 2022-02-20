// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrain;

public class DriveSubsystem extends SubsystemBase {
  
  private final MotorControllerGroup MG1;
  private final MotorControllerGroup MG2;

  private final List<CANSparkMax> motors;
  private final Set<RelativeEncoder> mg1Encoders = new HashSet<>();
  private final Set<RelativeEncoder> mg2Encoders = new HashSet<>();

  private final DifferentialDrive differentialDrive;


  private final AHRS navx = new AHRS(SPI.Port.kMXP);


  private static Function<Integer,CANSparkMax> createMotor = (id) ->
  {
    CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(DriveTrain.timeToFullSpeed);
    motor.setIdleMode(IdleMode.kCoast);

    RelativeEncoder maxEncoder = motor.getEncoder();
    maxEncoder.setPositionConversionFactor(DriveTrain.motorRotationsToInches);

    return motor;
  };

  public DriveSubsystem(List<Integer> mg1IDs, List<Integer> mg2IDs) {
   
    super();
  
    //little code du[plication but this will build all the motors and add their configuration
    motors = mg1IDs.stream()
    .map(id -> {
      CANSparkMax max = createMotor.apply(id);
      mg1Encoders.add(max.getEncoder());
      return max;
    })
    .collect(Collectors.toList());
    MG1 = new MotorControllerGroup((MotorController[])motors.toArray());


    List<CANSparkMax> mg2Motors =
      mg2IDs.stream()
      .map(id -> {
        CANSparkMax max = createMotor.apply(id);
        mg2Encoders.add(max.getEncoder());
        max.setInverted(true);
        return max;
    })
    .collect(Collectors.toList());

    motors.addAll(mg2Motors);
    MG2 = new MotorControllerGroup((MotorController[])mg2Motors.toArray());

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
    Stream.of(mg1Encoders, mg2Encoders)
        .forEach(encoders -> encoders.stream().map(encoder -> encoder.setPosition(0)));
  }

  public double getMg1Position(){
    return mg1Encoders.stream().map(RelativeEncoder::getPosition).reduce(0.0, Double::sum, Double::sum)/2.0;
  }

  public double getMg2Position() {
    return mg2Encoders.stream().map(RelativeEncoder::getPosition).reduce(0.0, Double::sum, Double::sum)/2.0;
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
