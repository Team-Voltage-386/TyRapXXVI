package frc.robot.constants.jr;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIOPhotonVision.CameraConfig;

public final class VisionConstants {

  // prevent instantiation
  private VisionConstants() {}

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double maxAmbiguity = 0.3;

  public static final double maxZError = 0.75;

  public static final CameraConfig[] cameraConfigs =
      new CameraConfig[] {
        new CameraConfig("camera_0", new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)))
      };

  public static final double[] cameraStdDevFactors = new double[] {1.0};

  // Meters
  public static final double linearStdDevBaseline = 0.02;

  public static final double angularStdDevBaseline = 0.06;

  public static final double linearStdDevMegatag2Factor = 0.5;

  public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

  // For CenterOnTag
  public static final double minXVelocity = 0.1;
  public static final double maxXVelocity = 1.0;
  public static final double minYVelocity = 0.1;
  public static final double maxYVelocity = 1.0;
  public static final double minAngVelocityDPS = 0;
  public static final double maxAngVelocityDPS = 20;
  public static final double xDisThreshold = 0.03;
  public static final double yDisThreshold = 0.03;
  public static final double rotThreshold = 1.0;
  public static final double azimuthFieldOfViewDeg = 29.0;
  public static final double xOffset = 0.5;
  public static final double yOffset = 0.0;
  public static final double maxAngAccMSS = 8;
  public static final double maxAngDccMSS = 16;
  // For ProportionalController & DriveDistance as well
  public static final double maxAccMSS = 4;
  public static final double maxDccMSS = 3;
  public static final double minVelocity = 0.15;
  public static final double maxVelocity = 5.0;
  public static final double offset = 0.0;
  public static final double proportion = 2;
  public static final double threshold = .02;
  // For DriveOffset
  public static final double driveOffsetXOffset = 0.5;
  public static final double driveOffsetYOffset = 0.3;
  public static final double driveOffsetMaxAccMSS = 1.0;
  public static final double driveOffsetMaxDccMSS = 3;
  public static final double driveOffsetMinVel = 0.05;
  public static final double driveOffsetMaxVel = 1.5;
  public static final double driveOffsetAngleErrorRadians = 0.015;
  public static final double driveOffsetRangeMThreshold = 0.02;
  // Camera Positioning
  public static final double cameraOffsetForwardM = 0.19;
}
