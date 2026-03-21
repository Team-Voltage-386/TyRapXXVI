package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIONull implements VisionIO {

  /**
   * Creates a new VisionIONull.
   */
  public VisionIONull(VisionIOPhotonVision.CameraConfig conf, Supplier<Pose2d> poseSupplier) {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {}
}
