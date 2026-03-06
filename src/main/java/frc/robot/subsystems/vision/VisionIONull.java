package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIONull extends VisionIOPhotonVision {

  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new VisionIONull.
   *
   * @param conf The configuration of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIONull(VisionIOPhotonVision.CameraConfig conf, Supplier<Pose2d> poseSupplier) {
    super(conf);
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
