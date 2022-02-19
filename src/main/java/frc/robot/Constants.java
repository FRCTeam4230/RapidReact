// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class DriveTrain {
    public static final double motorRotationsToWheelRotations = 1 / 10.71;
    public static final double wheelRotationsToInches = Math.PI * 6;
    // public static final double motorRotationsToInches = motorRotationsToWheelRotations * wheelRotationsToInches;
    public static final double motorRotationsToInches = 100 / 56.17;

    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double timeToFullSpeed = 0.5;
  }

  public final class Intake {
    public final class Limits {
      public static final double up = 1;
    }

    public static final double armSpeed = 0.1;
    public static final double speed = 0.1;
  }

  public final class Climber {
    public static final double highLimit = 10;

    public static final double speed = 0.1;
  }

  public static final class MotorIDs {
    public static final int arm = 7;
    public static final int intake = 8;
    public static final int leftClimber = 5;
    public static final int rightClimber = 6;
    public static final int[] dirveGroup1 = { 1, 2 };
    public static final int[] driveGroup2 = { 3, 4 };
  }

  public static class DigitalIOIDs {
    public static final int leftClimber = 5;
    public static final int rightClimber = 7;
    public static final int lowerArmLimit = 9;
  }
}
