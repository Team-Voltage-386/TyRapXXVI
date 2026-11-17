package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.jr.TurretConstants;

/** Shared yaw rules for hardware, manual commands, auto aim, and simulation. */
public final class TurretYawLimits {
  private TurretYawLimits() {}

  /** Resolve a heading to a target in the non-wrapping, inclusive 20-to-270-degree interval. */
  public static double limitDegrees(double degrees) {
    if (!Double.isFinite(degrees)) {
      return TurretConstants.turretStartupAngleDeg;
    }
    double heading = ((degrees % 360.0) + 360.0) % 360.0;
    if (heading >= TurretConstants.turretMinAngleDeg
        && heading <= TurretConstants.turretMaxAngleDeg) {
      return heading;
    }
    double lowerDistance =
        Math.abs(Math.IEEEremainder(heading - TurretConstants.turretMinAngleDeg, 360.0));
    double upperDistance =
        Math.abs(Math.IEEEremainder(heading - TurretConstants.turretMaxAngleDeg, 360.0));
    // Choose the lower endpoint on an exact tie at the dead-zone center.
    return lowerDistance <= upperDistance
        ? TurretConstants.turretMinAngleDeg
        : TurretConstants.turretMaxAngleDeg;
  }

  /** Positive voltage increases encoder yaw, as in the existing turret controller. */
  public static double limitVoltage(
      double positionRot, double volts, boolean lowerSwitch, boolean upperSwitch) {
    if (!Double.isFinite(positionRot) || !Double.isFinite(volts)) {
      return 0.0;
    }
    if ((positionRot <= TurretConstants.turretMinAngleRot || lowerSwitch) && volts < 0.0) {
      return 0.0;
    }
    if ((positionRot >= TurretConstants.turretMaxAngleRot || upperSwitch) && volts > 0.0) {
      return 0.0;
    }
    return MathUtil.clamp(volts, -TurretConstants.maxYawVoltage, TurretConstants.maxYawVoltage);
  }
}
