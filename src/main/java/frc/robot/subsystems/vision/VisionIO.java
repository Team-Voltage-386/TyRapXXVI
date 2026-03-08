package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  class VisionIOInputs {

    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d(), false, new Pose3d(), 0.0);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  record TargetObservation(
      Rotation2d tx, Rotation2d ty, boolean is3dValid, Pose3d tagPose, double zThetaDeg) {}

  /** Represents a robot pose sample used for pose estimation. */
  record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      boolean useCamera,
      PoseObservationType type) {}

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
