package frc.robot.constants.jr;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.VisionIOPhotonVision.CameraConfig;

public final class VisionConstants {

  // prevent instantiation
  private VisionConstants() {}

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double maxAmbiguity;

  public static final double maxZError;

  public static final CameraConfig[] cameraConfigs;

  public static final double[] cameraStdDevFactors;

  // Meters
  public static final double linearStdDevBaseline;

  public static final double angularStdDevBaseline;

  public static final double linearStdDevMegatag2Factor;

  public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

  // For CenterOnTag
  public static final double minXVelocity;
  public static final double maxXVelocity;
  public static final double minYVelocity;
  public static final double maxYVelocity;
  public static final double minAngVelocityDPS;
  public static final double maxAngVelocityDPS;
  public static final double xDisThreshold;
  public static final double yDisThreshold;
  public static final double rotThreshold;
  public static final double azimuthFieldOfViewDeg;
  public static final double xOffset;
  public static final double yOffset;
  public static final double maxAngAccMSS;
  public static final double maxAngDccMSS;
  // For ProportionalController & DriveDistance as well
  public static final double maxAccMSS;
  public static final double maxDccMSS;
  public static final double minVelocity;
  public static final double maxVelocity;
  public static final double offset;
  public static final double proportion;
  public static final double threshold;
  // For DriveOffset
  public static final double driveOffsetXOffset;
  public static final double driveOffsetYOffset;
  public static final double driveOffsetMaxAccMSS;
  public static final double driveOffsetMaxDccMSS;
  public static final double driveOffsetMinVel;
  public static final double driveOffsetMaxVel;
  public static final double driveOffsetAngleErrorRadians;
  public static final double driveOffsetRangeMThreshold;
  // Camera Positioning
  public static final double cameraOffsetForwardM;

  static {
    if (DriveConstants.isReefscape) {
      maxAmbiguity = 0.3;

      maxZError = 0.75;
      // 0.7366 front to back meters 0.6096 side to side

      cameraConfigs =
          new CameraConfig[] {
            new CameraConfig(
                "frontleft",
                new Transform3d(
                    0.7366 / 2,
                    0.6096 / 2,
                    0.2,
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(45)))),
            new CameraConfig(
                "frontright",
                new Transform3d(
                    0.7366 / 2,
                    -0.6096 / 2,
                    0.3,
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(315)))),
            new CameraConfig(
                "backleft",
                new Transform3d(
                    -0.7366 / 2,
                    0.6096 / 2,
                    0.2,
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(135)))),
            new CameraConfig(
                "backright",
                new Transform3d(
                    -0.7366 / 2,
                    -0.6096 / 2,
                    0.3,
                    new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(225)))),
          };

      cameraStdDevFactors = new double[] {1.0};

      // Meters
      linearStdDevBaseline = 0.02;

      angularStdDevBaseline = 0.06;

      linearStdDevMegatag2Factor = 0.5;

      // For CenterOnTag
      minXVelocity = 0.1;
      maxXVelocity = 1.0;
      minYVelocity = 0.1;
      maxYVelocity = 1.0;
      minAngVelocityDPS = 0;
      maxAngVelocityDPS = 20;
      xDisThreshold = 0.03;
      yDisThreshold = 0.03;
      rotThreshold = 1.0;
      azimuthFieldOfViewDeg = 29.0;
      xOffset = 0.5;
      yOffset = 0.0;
      maxAngAccMSS = 8;
      maxAngDccMSS = 16;
      // For ProportionalController & DriveDistance as well
      maxAccMSS = 4;
      maxDccMSS = 3;
      minVelocity = 0.15;
      maxVelocity = 5.0;
      offset = 0.0;
      proportion = 2;
      threshold = .02;
      // For DriveOffset
      driveOffsetXOffset = 0.5;
      driveOffsetYOffset = 0.3;
      driveOffsetMaxAccMSS = 1.0;
      driveOffsetMaxDccMSS = 3;
      driveOffsetMinVel = 0.05;
      driveOffsetMaxVel = 1.5;
      driveOffsetAngleErrorRadians = 0.015;
      driveOffsetRangeMThreshold = 0.02;
      // Camera Positioning
      cameraOffsetForwardM = 0.19;
    } else {
      maxAmbiguity = 0.3;

      maxZError = 0.75;

      cameraConfigs =
          new CameraConfig[] {
            new CameraConfig(
                "camera_0", new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)))
          };

      cameraStdDevFactors = new double[] {1.0};

      // Meters
      linearStdDevBaseline = 0.02;

      angularStdDevBaseline = 0.06;

      linearStdDevMegatag2Factor = 0.5;

      // For CenterOnTag
      minXVelocity = 0.1;
      maxXVelocity = 1.0;
      minYVelocity = 0.1;
      maxYVelocity = 1.0;
      minAngVelocityDPS = 0;
      maxAngVelocityDPS = 20;
      xDisThreshold = 0.03;
      yDisThreshold = 0.03;
      rotThreshold = 1.0;
      azimuthFieldOfViewDeg = 29.0;
      xOffset = 0.5;
      yOffset = 0.0;
      maxAngAccMSS = 8;
      maxAngDccMSS = 16;
      // For ProportionalController & DriveDistance as well
      maxAccMSS = 4;
      maxDccMSS = 3;
      minVelocity = 0.15;
      maxVelocity = 5.0;
      offset = 0.0;
      proportion = 2;
      threshold = .02;
      // For DriveOffset
      driveOffsetXOffset = 0.5;
      driveOffsetYOffset = 0.3;
      driveOffsetMaxAccMSS = 1.0;
      driveOffsetMaxDccMSS = 3;
      driveOffsetMinVel = 0.05;
      driveOffsetMaxVel = 1.5;
      driveOffsetAngleErrorRadians = 0.015;
      driveOffsetRangeMThreshold = 0.02;
      // Camera Positioning
      cameraOffsetForwardM = 0.19;
    }
  }
}
