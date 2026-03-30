package frc.robot.constants.rebuilt;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public final class DriveConstants {

  // prevent instantiation
  private DriveConstants() {}

  public static final double wheelRadiusMeters = 0.040;
  public static final double driveMotorReduction =
      6.12; // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);
  ;

  public static final double trackWidth = 0.302 * 2;
  ;
  public static final double wheelBase = 0.264 * 2;

  public static final double robotMassKg = 68.0;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;

  public static final int pigeonCanId = 13;

  public static final Rotation2d[] zeroRotations =
      new Rotation2d[] {
        new Rotation2d(3.085 + Math.PI),
        new Rotation2d(-0.86),
        new Rotation2d(-1.972 + 2 * Math.PI),
        new Rotation2d(2.826)
      };

  public static final int[] driveCanIds =
      new int[] {10, 16, 12, 15}; // Front Left, Front Right, Back Left, Back Right

  public static final int[] turnCanIds = new int[] {47, 44, 21, 20};

  public static final int[] turnCancoderIds = new int[] {9, 12, 10, 11};

  public static final int driveCurrentLimit = 50;

  public static final int turnCurrentLimit = 20;
  public static final double driveKp = 0.0;

  public static final double driveKd = 0.0;

  public static final double driveKs = 0.119;
  public static final double driveKv = 0.125;

  // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;

  // Rotor RPM -> Wheel Rad/Sec
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

  public static final double turnKp = 4.5;

  public static final double turnKi = 0.0;

  public static final double turnKd = 0.0;
  public static final double turnPIDMinInput = 0; // Radians

  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians

  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  public static final boolean turnInverted = true;
  public static final boolean turnEncoderInverted = true;

  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  public static final PIDConstants translationPID = new PIDConstants(3.0, 0.0, 0.05);

  public static final PIDConstants rotationPID = new PIDConstants(4.0, 0.0, 0.0);

  // Meters/Sec
  public static final double maxSpeed = 4.45;

  public static final double odometryFrequency = 100.0;

  public static final RobotConfig ppRobotConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeed,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveCurrentLimit,
              1),
          moduleTranslations);

  public static final double minVelocity = 0.1;
  public static final double maxVelocity = 5.0;
  public static final double maxAccMss = 3;
  public static final double maxDccMss = 8;
  public static final double driveDistanceProp = 3;
  public static final double driveDistanceThreshold = 0.02;
}
