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
}
