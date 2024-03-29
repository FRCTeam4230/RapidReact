package frc.robot;

public final class Constants {
  public static final double MOTOR_RAMP_TIME = 0.3;

  public static final class ArmConstants {

    public static final double kP = 0.01;

  }

  public static final class DriveDistanceParams {
    public static final double kP = 0.03;
    public static final double kI = 0;
    public static final double kD = 0.005;

    public static final double baseSpeed = 0.02;

    public static final double tolerance = 1;
    public static final double velocityTolerance = 0.2;
  }

  public static final class TurnCommandParams {
    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double baseSpeed = 0.02;

    public static final double tolerance = 0.25;
    public static final double velocityTolerance = 0.2;
  }

  public final class DriveTrain {
    public static final double motorRotationsToWheelRotations = 1 / 10.71;
    public static final double wheelRotationsToInches = Math.PI * 6;
    public static final double motorRotationsToInches = 72 / 40.687;

    public static final double timeToFullSpeed = 0.5;
    public static final double moveMult = 0.8;
    public static final double turnMult = 0.6;

    public static final double moveMult2 = 0.9;
    public static final double turnMult2 = 0.7;

    public static final double accelTime = 0.7;
  }

  public static class Intake {
    public final class Limits {
      public static final double up = 21.3;
    }

    public static final double downArmSpeed = -0.2;
    public static final double upArmSpeed = 0.3;
    public static double holdSpeed = 0.04;
    public static double upHoldSpeed = 0.01;

    public static final double speed = 1;
  }

  public final class Climber {
    public static final int resetDirection = -1;
    public static final double highLimit = 190;

    public static final double resetRotations = 15;

    public static final double speed = 0.9;
    public static final double rightSpeedMult = 4.0 / 5.0;
    public static final double leftSpeedMult = 1;
  }

  public class Autonomous {
    public static final double defaultTaxiDistance = 8 * 12;
  }

  /*
  public static final class MotorIDs {
    public static final int arm = 7;
    public static final int intake = 8;
    public static final int leftClimber = 5;
    public static final int rightClimber = 6;
    public static final List<Integer> dirveGroup1 = Arrays.asList(1,2);
    public static final List<Integer> driveGroup2 = Arrays.asList(3, 4 );
  */

  public enum MotorID {
    ARM(7), INTAKE(8), LEFT_CLIMBER(6), RIGHT_CLIMBER(5), MG1_1(1), MG1_2(2), MG2_1(3), MG2_2(4);

    private Integer id;

    private MotorID(Integer id) {
      this.id = id;
    }

    public Integer getId() {
      return id;
    }
  }

  public static class DigitalIOIDs {
    public static final int leftClimber = 5;
    public static final int rightClimber = 3;
    public static final int lowerArmLimit = 9;
    public static final int upperArmLimit = 7;
  }
}
