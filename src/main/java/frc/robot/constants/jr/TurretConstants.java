package frc.robot.constants.jr;

public class TurretConstants {

  public static final double turretRPMToMetersPerSecond =
      (1.0 / 6000) * 12; // 12 meters per second @ 6000 RPM
  public static final int turretCanId = 52;
  public static final double zeroRotRadians = 0;

  public static final double gearRatioPerRot = 1.0 / 22.57;

  // Ks is maximum voltage that does not move the motor. Just try voltages until the motor stops
  // moving.
  public static final double turretKs = 0.13; // volts

  // This is found by running the motor at an arbitrary voltage (e.g., 3.3V) and measuring the
  // velocity reported by the encoder.
  // You then subtract the Ks from the voltage and divide by the velocity to get Kv.
  public static final double turretKv = (3.3 - turretKs) / 68.25; // Volts per (rpm)
}
