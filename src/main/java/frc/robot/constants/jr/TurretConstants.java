package frc.robot.constants.jr;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
  public static final int turretYawCanId = 52;
  public static final int turretPitchCanId = 45;
  public static final double zeroRotRadians = 0;

  public static final double gearRatioPerRot = 1.0 / 22.57;

  public static final double offsetX = -0.16;
  public static final double offsetY = 0.045;
  public static final Translation2d turretPosition = new Translation2d(offsetX, offsetY);

  public static final double turretMaxAngleRot = Units.degreesToRotations(120);
  public static final double turretMinAngleRot = -turretMaxAngleRot;

  public static final double turretMaxHoodAngle = 62;
  public static final double turretMinHoodAngle = 40;
  public static final double turretHoodAngleRange = turretMaxHoodAngle - turretMinHoodAngle;

  public static final double maxHoodSetpoint = 0.71;

  // Ks is maximum voltage that does not move the motor. Just try voltages until the motor stops
  // moving.
  public static final double turretKs = 0.13; // volts

  // This is found by running the motor at an arbitrary voltage (e.g., 3.3V) and measuring the
  // velocity reported by the encoder.
  // You then subtract the Ks from the voltage and divide by the velocity to get Kv.
  public static final double turretKv = (3.3 - turretKs) / 68.25; // Volts per (rpm)

  public static final int flywheelCanId = 13;
  public static final double flywheelKs = 0.11;
  public static final double flywheelKv = 0.00204;
  public static final double flywheelKa = 0.006;

  public static final double shooterWheelRadiusMeters = 0.05;

  public static final double turretRPMToMetersPerSecond =
      (2 * Math.PI * TurretConstants.shooterWheelRadiusMeters) / 60;
}
