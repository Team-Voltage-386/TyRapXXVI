package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.jr.VisionConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {

  public static class CameraConfig {

    public CameraConfig(String name, Transform3d robotToCam) {
      this.cameraName = name;
      this.robotToCamera = robotToCam;
    }

    /**
     * The configured name of the camera
     */
    public String cameraName;

    /**
     * The 3D position of the camera relative to the robot.
     */
    public Transform3d robotToCamera;
  }

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected boolean useCamera = false;

  /**
   * Creates a new VisionIOPhotonVision.
   */
  public VisionIOPhotonVision(CameraConfig conf) {
    camera = new PhotonCamera(conf.cameraName);
    this.robotToCamera = conf.robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (var target : targets) {
        int climbTags = target.getFiducialId();
        if (climbTags == 31 || climbTags == 32 || climbTags == 15 || climbTags == 16) {
          useCamera = true;
        } else useCamera = false;
      }
      if (result.hasTargets()) {
        double zThetaDeg =
            result
                .getBestTarget()
                .getBestCameraToTarget()
                .getRotation()
                .toRotation2d()
                .getDegrees();
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()),
                true,
                new Pose3d(
                    result.getBestTarget().getBestCameraToTarget().getTranslation(),
                    new Rotation3d()),
                zThetaDeg);
      } else {
        inputs.latestTargetObservation =
            new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, false, new Pose3d(), 0.0);
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                useCamera, // Only use camera when the best target is tag 31 or 32 (for climbing)
                PoseObservationType.PHOTONVISION)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = VisionConstants.aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  useCamera,
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
