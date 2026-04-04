package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class Passing extends ShootingConstants {

  public Passing() {

    minDistance = 2.0;
    maxDistance = 10.0;
    phaseDelay = 0.03;

    // max is 62 degrees
    launchHoodAngleMap.put(2.0, Rotation2d.fromDegrees(52.0));
    launchHoodAngleMap.put(3.0, Rotation2d.fromDegrees(50.0));
    launchHoodAngleMap.put(4.5, Rotation2d.fromDegrees(40.0));
    launchHoodAngleMap.put(6.0, Rotation2d.fromDegrees(40.0));
    launchHoodAngleMap.put(8.0, Rotation2d.fromDegrees(40.0));
    // launchHoodAngleMap.put(10.0, Rotation2d.fromDegrees(40.0));

    launchFlywheelSpeedMap.put(2.0, 2000.0); // RPM
    launchFlywheelSpeedMap.put(3.0, 2000.0); // RPM
    launchFlywheelSpeedMap.put(4.5, 2550.0);
    launchFlywheelSpeedMap.put(6.0, 3000.0);
    launchFlywheelSpeedMap.put(8.0, 3750.0);
    // launchFlywheelSpeedMap.put(10.0, 4500.0);

    timeOfFlightMap.put(3.11, 0.92);
    timeOfFlightMap.put(4.0, 0.96);
    timeOfFlightMap.put(5.5, 1.03);
    timeOfFlightMap.put(8.1, 1.55);
    timeOfFlightMap.put(9.11, 1.61);
  }
}
