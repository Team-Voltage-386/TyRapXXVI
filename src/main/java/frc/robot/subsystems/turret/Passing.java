package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class Passing extends ShootingConstants {

  public Passing() {

    minDistance = 2.0;
    maxDistance = 10.0;
    phaseDelay = 0.03;

    // max is 62 degrees

    // need to
    launchHoodAngleMap.put(2.0, Rotation2d.fromDegrees(52.0));
    launchHoodAngleMap.put(3.0, Rotation2d.fromDegrees(50.0));
    launchHoodAngleMap.put(4.5, Rotation2d.fromDegrees(40.0));
    launchHoodAngleMap.put(6.0, Rotation2d.fromDegrees(40.0));
    launchHoodAngleMap.put(8.0, Rotation2d.fromDegrees(40.0));
    //launchHoodAngleMap.put(10.0, Rotation2d.fromDegrees(40.0));
    // test 10

    // get TOF
    launchFlywheelSpeedMap.put(2.0, 2000.0); // RPM
    launchFlywheelSpeedMap.put(3.0, 2000.0); // RPM
    launchFlywheelSpeedMap.put(4.5, 2550.0);
    launchFlywheelSpeedMap.put(6.0, 3000.0);
    launchFlywheelSpeedMap.put(8.0, 3750.0);
    //launchFlywheelSpeedMap.put(10.0, 4500.0);

    timeOfFlightMap.put(1.669, 0.88);
    timeOfFlightMap.put(2.233, 0.9275);
    timeOfFlightMap.put(2.823, 1.007);
    timeOfFlightMap.put(3.782, 1.057);
    timeOfFlightMap.put(4.75, 1.167);
    timeOfFlightMap.put(5.0, 1.197);
  }
}
