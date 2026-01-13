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

  public static final double driveKp = frc.robot.constants.jr.DriveConstants.driveKp;
  public static final double driveKd = frc.robot.constants.jr.DriveConstants.driveKd;
  public static final double driveKs = frc.robot.constants.jr.DriveConstants.driveKs;
  public static final double driveKv = frc.robot.constants.jr.DriveConstants.driveKv;

  public static final double turnKp = frc.robot.constants.jr.DriveConstants.turnKp;
  public static final double turnKd = frc.robot.constants.jr.DriveConstants.turnKd;

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

  public static final double odometryFrequency = 100.0;

  public static final double wheelRadiusMeters =
      frc.robot.constants.jr.DriveConstants.wheelRadiusMeters;
}
