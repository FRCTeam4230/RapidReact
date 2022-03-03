// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveDistanceParams;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem driveSubsystem;

  private final PIDController mg1PidController = new PIDController(DriveDistanceParams.kP, DriveDistanceParams.kI,
      DriveDistanceParams.kD);
  private final PIDController mg2PidController = new PIDController(DriveDistanceParams.kP, DriveDistanceParams.kI,
      DriveDistanceParams.kD);

  private Double distance;

  public Double getDistance() {
    return distance;
  }

  private DriveDistance setDistance(Double distance) {
    this.distance = distance;
    return this;
  }

  private Double baseSpeed = DriveDistanceParams.baseSpeed;

  private DriveDistance(DriveSubsystem driveSubsystem) {
    this(driveSubsystem, 0.5);
  }

  /** Creates a new DriveDistance. */
  private DriveDistance(DriveSubsystem driveSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    mg1PidController.setTolerance(DriveDistanceParams.tolerance, DriveDistanceParams.tolerance);
    mg2PidController.setTolerance(DriveDistanceParams.tolerance, DriveDistanceParams.tolerance);

    this.distance = distance;

    SmartDashboard.putData("drive dist " + distance, this);
  }

  public static final DriveDistance create(DriveSubsystem driveSubsystem) {
    return new DriveDistance(driveSubsystem);
  }

  public static final DriveDistance create(DriveSubsystem driveSubsystem, Double distanceAsInches) {
    return new DriveDistance(driveSubsystem, distanceAsInches);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mg1PidController.setSetpoint(driveSubsystem.getMg1Position() + distance);
    mg2PidController.setSetpoint(driveSubsystem.getMg2Position() + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mg1Output = mg1PidController.calculate(driveSubsystem.getMg1Position());
    double mg2Output = mg2PidController.calculate(driveSubsystem.getMg2Position());

    mg1Output += Math.copySign(baseSpeed, mg1Output);
    mg2Output += Math.copySign(baseSpeed, mg2Output);

    mg1Output = MathUtil.clamp(mg1Output, -.7, .7);
    mg2Output = MathUtil.clamp(mg2Output, -.7, .7);

    driveSubsystem.setSpeeds(mg1Output, mg2Output);
    NetworkTableInstance.getDefault().getEntry("mg1Output").setDouble(mg1Output);
    NetworkTableInstance.getDefault().getEntry("mg2Output").setDouble(mg2Output);

    NetworkTableInstance.getDefault().getEntry("1at setpoint").setBoolean(mg1PidController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mg1PidController.atSetpoint() || mg2PidController.atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("kP", mg1PidController::getP, kP -> {
      mg1PidController.setP(kP);
      mg2PidController.setP(kP);
    });

    builder.addDoubleProperty("kD", mg1PidController::getD, kD -> {
      mg1PidController.setD(kD);
      mg2PidController.setD(kD);
    });

    builder.addDoubleProperty("kI", mg1PidController::getI, kI -> {
      mg1PidController.setI(kI);
      mg2PidController.setI(kI);
    });

    builder.addDoubleProperty("base speed", () -> baseSpeed, s -> baseSpeed = s);
    builder.addDoubleProperty("distance", () -> distance, s -> distance = s);
  }
}
