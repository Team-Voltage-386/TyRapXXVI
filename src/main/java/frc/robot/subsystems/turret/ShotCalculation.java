package frc.robot.subsystems.turret;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.constants.rebuilt.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TuningUtil;
import org.littletonrobotics.junction.Logger;

public class ShotCalculation {
  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage(5);
  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(5);

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;
  TuningUtil addRPM = new TuningUtil("/Tuning/turret/addRPM", 0.0);

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      Pose2d lookaheadPose) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  Drive dt;
  private Translation2d target;

  public static final Scoring scoring = new Scoring();

  public static final Passing passing = new Passing();

  protected ShootingConstants shootingConstants = scoring;

  public ShotCalculation(Drive dt) {
    this.dt = dt;
    setTarget(Constants.blueHubPose.toTranslation2d(), true);
  }

  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d estimatedPose = dt.getPose();
    ChassisSpeeds robotRelativeVelocity = dt.getChassisSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * shootingConstants.phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * shootingConstants.phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * shootingConstants.phaseDelay));

    Pose2d turretPosition =
        estimatedPose.transformBy(
            new Transform2d(TurretConstants.turretPosition, Rotation2d.kZero));
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, dt.getPose().getRotation());
    double robotAngle = estimatedPose.getRotation().getRadians();

    // doesn't account for rotational velocity of the turret so its not exactly true
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.turretPosition.getY() * Math.cos(robotAngle)
                    - TurretConstants.turretPosition.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (TurretConstants.turretPosition.getX() * Math.cos(robotAngle)
                    - TurretConstants.turretPosition.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = shootingConstants.timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());

      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    // lookaheadPose = boundLookahead(lookaheadPose);
    lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle =
        shootingConstants.launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= shootingConstants.minDistance
                && lookaheadTurretToTargetDistance <= shootingConstants.maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            shootingConstants.launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)
                + addRPM.getValue(),
            lookaheadPose);

    // Log calculated values
    Logger.recordOutput(
        "ShotCalculation/isValid",
        lookaheadTurretToTargetDistance >= shootingConstants.minDistance
            && lookaheadTurretToTargetDistance <= shootingConstants.maxDistance);
    Logger.recordOutput("ShotCalculation/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculation/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public void setTarget(Translation2d target, boolean isScoring) {
    this.target = target;
    if (isScoring) {
      shootingConstants = scoring;
    } else {
      shootingConstants = passing;
    }
  }

  public boolean boundLookahead(Pose2d pose) {
    if (pose.getX() > 16.5) {
      return false;
    }
    if (pose.getX() < 0.5) {
      return false;
    }
    if (pose.getY() > 7.8) {
      return false;
    }
    if (pose.getY() < 0.67) {
      return false;
    }
    return true;
  }
}
