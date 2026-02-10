package frc.robot.subsystems.turret;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.subsystems.drive.Drive;
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

  private Drive dt;
  private Translation2d target;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.5;
    maxDistance = 6.7;
    phaseDelay = 0.03; // figure this out idk ask chief delphi

    launchHoodAngleMap.put(1.5, Rotation2d.fromDegrees(67.5));
    launchHoodAngleMap.put(2.5, Rotation2d.fromDegrees(61.1));
    launchHoodAngleMap.put(3.5, Rotation2d.fromDegrees(56.67));
    launchHoodAngleMap.put(4.5, Rotation2d.fromDegrees(56.0));
    launchHoodAngleMap.put(6.5, Rotation2d.fromDegrees(54.1));
    launchHoodAngleMap.put(6.7, Rotation2d.fromDegrees(55.8));

    launchFlywheelSpeedMap.put(1.5, 5.65); // these are in m/s
    launchFlywheelSpeedMap.put(2.5, 6.34);
    launchFlywheelSpeedMap.put(3.5, 7.0);
    launchFlywheelSpeedMap.put(4.5, 7.66);
    launchFlywheelSpeedMap.put(5.5, 8.25);
    launchFlywheelSpeedMap.put(6.7, 9.0);

    timeOfFlightMap.put(1.5, 0.69);
    timeOfFlightMap.put(2.5, 0.81);
    timeOfFlightMap.put(3.5, 0.91);
    timeOfFlightMap.put(4.5, 1.05);
    timeOfFlightMap.put(5.5, 1.13);
    timeOfFlightMap.put(6.7, 1.32);
  }

  public ShotCalculation(Drive dt) {
    this.dt = dt;
    setTarget(Constants.blueHubPose.toTranslation2d());
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
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // TODO: use hub pose based on alliance, i tried to but it ending up messing the distance values
    // for some reason
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
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
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
    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance),
            lookaheadPose);

    // Log calculated values
    Logger.recordOutput(
        "ShotCalculation/isValid",
        lookaheadTurretToTargetDistance >= minDistance
            && lookaheadTurretToTargetDistance <= maxDistance);
    Logger.recordOutput("ShotCalculation/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculation/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public void setTarget(Translation2d target) {
    this.target = target;
  }

  public boolean boundLookahead(Pose2d pose) {
    if (pose.getX() > 16.0) {
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
