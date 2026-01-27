package frc.robot.constants.sim;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;

// ideally all of these constants are the same as their real counterparts,
// but they can be changed if needed
public final class DriveConstants {

  // prevent instantiation
  private DriveConstants() {}

  public static final int driveCurrentLimit =
      frc.robot.constants.jr.DriveConstants.driveCurrentLimit;
  public static final int turnCurrentLimit = frc.robot.constants.jr.DriveConstants.turnCurrentLimit;

  public static final double driveKp = 0.05;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.0789;

  public static final double turnKp = 8.0;
  public static final double turnKd = 0.0;

  public static final Translation2d[] moduleTranslations =
      frc.robot.constants.jr.DriveConstants.moduleTranslations;

  public static final double driveBaseRadius =
      frc.robot.constants.jr.DriveConstants.driveBaseRadius;
  public static final double maxSpeed = frc.robot.constants.jr.DriveConstants.maxSpeed;

  public static final PIDConstants translationPID =
      frc.robot.constants.jr.DriveConstants.translationPID;
  public static final PIDConstants rotationPID = frc.robot.constants.jr.DriveConstants.rotationPID;

  public static final RobotConfig ppRobotConfig =
      frc.robot.constants.jr.DriveConstants.ppRobotConfig;

  public static final double odometryFrequency;

  public static final double wheelRadiusMeters =
      frc.robot.constants.jr.DriveConstants.wheelRadiusMeters;

  static {
    if (frc.robot.constants.jr.DriveConstants.isReefscape) {
      odometryFrequency = 100.0;
    } else {
      odometryFrequency = 100.0;
    }
  }
}
