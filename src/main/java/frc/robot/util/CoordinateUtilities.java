package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CoordinateUtilities {

  /**
   * Returns a WPILib transform representation of a position offset
   * expressed in range and angle using the FRC coordinate system
   *
   * @return Transform2d with x and y components reflecting the range and angle
   */
  public static Transform2d rangeAngleToTransform(double rangeM, double angleDeg) {
    double x = rangeM * Math.cos(Math.toRadians(angleDeg));
    double y = rangeM * Math.sin(Math.toRadians(angleDeg));
    return new Transform2d(x, y, new Rotation2d());
  }

  /**
   * Returns the distance in meters between two 2D positions
   *
   * @return Distance in meters
   */
  public static double distanceTo(Pose2d first, Pose2d second) {
    return Math.sqrt(
        Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2));
  }

  /**
   * Returns the bearing angle in degrees between two positions. The rotation
   * elements of the poses are not considered
   *
   * @return Bearing in degrees from first to second pos position
   */
  public static double bearingTo(Pose2d first, Pose2d second) {
    return Math.toDegrees(Math.atan2(second.getY() - first.getY(), second.getX() - first.getX()));
  }

  public static double robotBearingTo(Pose2d first, Pose2d second) {
    double fieldBearing = bearingTo(first, second);
    double robotBearing = first.getRotation().getDegrees();
    return fieldBearing - robotBearing;
  }

  /**
   * Converts a course and speed to its x and y components. The returned value will
   * have zero angular velocity
   *
   * @return ChassisSpeeds object with x and y velocities for the specified course and speed
   */
  public static ChassisSpeeds courseSpeedToLinearVelocity(
      double courseDeg, double speedMetersPerSecond) {
    return new ChassisSpeeds(
        speedMetersPerSecond * Math.cos(Math.toRadians(courseDeg)),
        speedMetersPerSecond * Math.sin(Math.toRadians(courseDeg)),
        0.0);
  }

  public static double getChassisMagnitude(ChassisSpeeds speeds) {
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
  }
}
