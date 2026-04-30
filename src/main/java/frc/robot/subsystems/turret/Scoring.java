package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class Scoring extends ShootingConstants {

  public Scoring() {

    minDistance = 1.5;
    maxDistance = 6.7;
    phaseDelay = 0.03;

    // max is 62 degrees

    launchHoodAngleMap.put(1.602, Rotation2d.fromDegrees(62.0));
    launchHoodAngleMap.put(2.602, Rotation2d.fromDegrees(57.0));
    launchHoodAngleMap.put(3.602, Rotation2d.fromDegrees(53.0));
    launchHoodAngleMap.put(4.602, Rotation2d.fromDegrees(51.0));
    launchHoodAngleMap.put(4.83, Rotation2d.fromDegrees(50.0));
    launchHoodAngleMap.put(5.54, Rotation2d.fromDegrees(48.0));

    launchFlywheelSpeedMap.put(1.602, 2262.0); // RPM
    launchFlywheelSpeedMap.put(2.602, 2400.0);
    launchFlywheelSpeedMap.put(3.602, 2650.0 + 40);
    launchFlywheelSpeedMap.put(4.602, 2925.0 + 50 + 40);
    launchFlywheelSpeedMap.put(4.83, 3042.0 + 60 + 40);
    launchFlywheelSpeedMap.put(5.54, 3200.0 + 70 + 40);
    launchFlywheelSpeedMap.put(6.8, 3500.0);

    timeOfFlightMap.put(1.669, 0.88);
    timeOfFlightMap.put(2.233, 0.9275);
    timeOfFlightMap.put(2.823, 1.007);
    timeOfFlightMap.put(3.782, 1.057);
    timeOfFlightMap.put(4.75, 1.167);
    timeOfFlightMap.put(5.54, 1.23);
    timeOfFlightMap.put(6.8, 1.4);
  }
}
