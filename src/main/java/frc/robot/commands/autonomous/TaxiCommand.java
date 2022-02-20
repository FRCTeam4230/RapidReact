// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class TaxiCommand {
  private TaxiCommand() {
  }

  public static Command create(DriveSubsystem driveSubsystem) {
    DriveDistance out = DriveDistance.create(driveSubsystem, Autonomous.defaultTaxiDistance * 12);

    SmartDashboard.putData("Taxi command", out);
    return out;
  }
}
