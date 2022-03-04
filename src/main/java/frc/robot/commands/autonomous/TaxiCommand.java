// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class TaxiCommand {
  private TaxiCommand() {
  }

  public static Command create(DriveSubsystem driveSubsystem) {
    CommandBase out = DriveDistance.create(driveSubsystem, Autonomous.defaultTaxiDistance).withName("Taxi command").withTimeout(8);

    SmartDashboard.putData(out);
    return out;
  }
}
