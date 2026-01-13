package frc.robot.constants.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.vision.VisionIOPhotonVision.CameraConfig;

public final class VisionConstants { // removed implements to expose only static values

  // prevent instantiation
  private VisionConstants() {}

  // static values replacing previous instance methods (names kept the same)
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double maxAmbiguity = frc.robot.constants.jr.VisionConstants.maxAmbiguity;
  public static final double maxZError = frc.robot.constants.jr.VisionConstants.maxZError;
  public static final double linearStdDevBaseline =
      frc.robot.constants.jr.VisionConstants.linearStdDevBaseline;
  public static final double angularStdDevBaseline =
      frc.robot.constants.jr.VisionConstants.angularStdDevBaseline;
  public static final double linearStdDevMegatag2Factor =
      frc.robot.constants.jr.VisionConstants.linearStdDevMegatag2Factor;
  public static final double angularStdDevMegatag2Factor =
      frc.robot.constants.jr.VisionConstants.angularStdDevMegatag2Factor;

  public static final double[] cameraStdDevFactors = new double[0];

  public static final CameraConfig[] cameraConfigs =
      frc.robot.constants.jr.VisionConstants.cameraConfigs;
}
