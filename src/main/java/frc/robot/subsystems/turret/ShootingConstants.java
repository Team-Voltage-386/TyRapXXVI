package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShootingConstants {

  protected double minDistance;
  protected double maxDistance;
  protected double phaseDelay;
  protected final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  protected final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  protected final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
}
