package frc.robot.constants.jr;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  public static final boolean isReefscape = true;

  // prevent instantiation
  private DriveConstants() {}

  public static final double wheelRadiusMeters;
  public static final double
      driveMotorReduction; // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox;
  public static final DCMotor turnGearbox;

  public static final double trackWidth;
  public static final double wheelBase;

  public static final double robotMassKg;
  public static final double robotMOI;
  public static final double wheelCOF;

  public static final int pigeonCanId;

  public static final Rotation2d[] zeroRotations;

  public static final int[] driveCanIds;

  public static final int[] turnCanIds;

  public static final int[] turnCancoderIds;

  public static final int driveCurrentLimit;

  public static final int turnCurrentLimit;

  public static final double driveKp;

  public static final double driveKd;

  public static final double driveKs;

  public static final double driveKv;

  // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderPositionFactor;

  // Rotor RPM -> Wheel Rad/Sec
  public static final double driveEncoderVelocityFactor;

  public static final double turnKp;

  public static final double turnKi;

  public static final double turnKd;

  public static final double turnPIDMinInput; // Radians

  public static final double turnPIDMaxInput; // Radians

  public static final double turnEncoderPositionFactor; // Rotations -> Radians

  public static final double turnEncoderVelocityFactor; // RPM -> Rad/Sec

  public static final boolean turnInverted;

  public static final boolean turnEncoderInverted;

  public static final Translation2d[] moduleTranslations;

  public static final double driveBaseRadius;

  public static final PIDConstants translationPID;

  public static final PIDConstants rotationPID;

  // Meters/Sec
  public static final double maxSpeed;

  public static final double odometryFrequency;

  public static final RobotConfig ppRobotConfig;

  static {
    if (isReefscape) {
      wheelRadiusMeters = 0.048;
      driveMotorReduction = 6.12; // MAXSwerve with 14 pinion teeth and 22 spur teeth
      driveGearbox = DCMotor.getNEO(1);
      turnGearbox = DCMotor.getNEO(1);

      trackWidth = Units.inchesToMeters(24.75);
      wheelBase = Units.inchesToMeters(24.75);

      robotMassKg = 24.088;
      robotMOI = 6.883;
      wheelCOF = 1.2;

      pigeonCanId = 2;

      zeroRotations =
          new Rotation2d[] {
            new Rotation2d(2.8), new Rotation2d(2.27), new Rotation2d(1.094), new Rotation2d(-0.08)
          };

      driveCanIds = new int[] {2, 4, 6, 8}; // Front Left, Front Right, Back Left, Back Right

      turnCanIds = new int[] {1, 3, 5, 7};

      turnCancoderIds = new int[] {22, 21, 23, 24};

      driveCurrentLimit = 60;

      turnCurrentLimit = 20;

      driveKp = 2.0;

      driveKd = 0.0;

      driveKs = 0.17;

      driveKv = 2.255;

      // Rotor Rotations -> Wheel Radians
      driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;

      // Rotor RPM -> Wheel Rad/Sec
      driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction;

      turnKp = 4.5;

      turnKi = 1.0;

      turnKd = 0.0;

      turnPIDMinInput = 0; // Radians

      turnPIDMaxInput = 2 * Math.PI; // Radians

      turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians

      turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

      turnInverted = true;

      turnEncoderInverted = true;

      moduleTranslations =
          new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
          };

      driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

      translationPID = new PIDConstants(2.0, 0.0, 0.05);

      rotationPID = new PIDConstants(4.5, 1.0, 0.0);

      // Meters/Sec
      maxSpeed = 5.3;

      odometryFrequency = 100.0;

      ppRobotConfig =
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

    } else {
      wheelRadiusMeters = 0.048;
      driveMotorReduction = 6.12; // MAXSwerve with 14 pinion teeth and 22 spur teeth
      driveGearbox = DCMotor.getNEO(1);
      turnGearbox = DCMotor.getNEO(1);

      trackWidth = Units.inchesToMeters(24.75);
      wheelBase = Units.inchesToMeters(24.75);

      robotMassKg = 24.088;
      robotMOI = 6.883;
      wheelCOF = 1.2;

      pigeonCanId = 13;

      zeroRotations =
          new Rotation2d[] {
            new Rotation2d(2.8), new Rotation2d(2.27), new Rotation2d(1.094), new Rotation2d(-0.08)
          };

      driveCanIds = new int[] {6, 52, 7, 2}; // Front Left, Front Right, Back Left, Back Right

      turnCanIds = new int[] {5, 8, 4, 3};

      turnCancoderIds = new int[] {11, 12, 10, 9};

      driveCurrentLimit = 60;

      turnCurrentLimit = 20;

      driveKp = 0.0;

      driveKd = 0.0;

      driveKs = 0.119;

      driveKv = 0.125;

      // Rotor Rotations -> Wheel Radians
      driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;

      // Rotor RPM -> Wheel Rad/Sec
      driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction;

      turnKp = 4.5;

      turnKi = 0.0;

      turnKd = 0.0;

      turnPIDMinInput = 0; // Radians

      turnPIDMaxInput = 2 * Math.PI; // Radians

      turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians

      turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

      turnInverted = true;

      turnEncoderInverted = true;

      moduleTranslations =
          new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
          };

      driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

      translationPID = new PIDConstants(3.0, 0.0, 0.05);

      rotationPID = new PIDConstants(4.0, 0.0, 0.0);

      // Meters/Sec
      maxSpeed = 5.45;

      odometryFrequency = 100.0;

      ppRobotConfig =
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
    }
  }
}
