package frc.robot.constants.jr;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
  public static final int turretYawCanId = 48;
  public static final int turretPitchCanId = 45;
  public static final double zeroRotRadians = 0;

  public static final double gearRatioPerRot = 1.0 / ((215.0 / 68.0) * 27.0);

  public static final double offsetX = -0.127;
  public static final double offsetY = -.099;
  public static final Translation2d turretPosition = new Translation2d(offsetX, offsetY);

  public static final double turretCenterOffsetDeg = 0.0;
  public static final Rotation2d turretCenterOffsetRot =
      Rotation2d.fromDegrees(turretCenterOffsetDeg);
  public static final double turretDeadZoneCenterDeg = 36.5;
  public static final double turretDeadZoneWidthDeg = 20.0;
  public static final double turretDeadZoneHalfWidthDeg = turretDeadZoneWidthDeg / 2.0;
  public static final double turretDeadZoneStartDeg =
      turretDeadZoneCenterDeg - turretDeadZoneHalfWidthDeg;
  public static final Rotation2d turretDeadZoneStartRot =
      Rotation2d.fromDegrees(turretDeadZoneStartDeg);
  public static final double turretDeadZoneEndDeg =
      turretDeadZoneCenterDeg + turretDeadZoneHalfWidthDeg;
  public static final Rotation2d turretDeadZoneEndRot =
      Rotation2d.fromDegrees(turretDeadZoneEndDeg);
  public static final double turretYawLimitMarginRot =
      0.01; // Margin to prevent hitting the limit switche
  public static final double turretMaxAngleRot =
      0.075 - turretYawLimitMarginRot; // Maximum counter clockwise rotation in rotations
  public static final double turretMinAngleRot =
      -.875 + turretYawLimitMarginRot; // Maximum clockwise rotation in rotations

  public static final double turretMaxRotationSpeedDegPerSec = 360.0;
  public static final double turretMaxRotationSpeedRotPerSec =
      turretMaxRotationSpeedDegPerSec / 360.0;
  public static final double turretMaxAccelDps2 = 720.0;
  public static final double turretMaxAccelRotPerSec2 = turretMaxAccelDps2 / 360.0;
  public static final double maxYawVoltage = 8.0;

  public static final double turretMaxHoodAngle = 62;
  public static final Rotation2d turretMaxHoodRot = Rotation2d.fromDegrees(turretMaxHoodAngle);
  public static final double turretMinHoodAngle = 40;
  public static final Rotation2d turretMinHoodRot = Rotation2d.fromDegrees(turretMinHoodAngle);
  public static final double turretHoodAngleRange = turretMaxHoodAngle - turretMinHoodAngle;

  public static final double maxHoodSetpoint = 0.92;
  public static final double hoodKP = 2.4;

  // Ks is maximum voltage that does not move the motor. Just try voltages until the motor stops
  // moving.
  public static final double turretKs = 0.5; // volts
  public static final double turretYawKp = 80.0; // volts per rotation
  public static final double turretYawKd = 0.0; // volts per rotation per second

  // This is found by running the motor at an arbitrary voltage (e.g., 3.3V) and measuring the
  // velocity reported by the encoder.
  // You then subtract the Ks from the voltage and divide by the velocity to get Kv.
  public static final double turretKv = 7.0; // Volts per (rpm)

  public static final int flywheelMasterCanId = 11;
  public static final int flywheelSlaveCanId = 19;
  public static final double flywheelKs = 0.0123;
  public static final double flywheelKv = 0.00182;
  public static final double flywheelKa = 0.006;
  public static final double manualShotSpeedRpm = 2270.0;
  public static final double manualTriggerOnThreshold = 0.15;
  public static final double flywheelRateLimit = 3300.0;

  public static final double shooterWheelRadiusMeters = 0.05;

  public static final double turretRPMToMetersPerSecond =
      (2 * Math.PI * TurretConstants.shooterWheelRadiusMeters) / 60;
}
