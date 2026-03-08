package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Translation3d blueHubPose = new Translation3d(4.5974, 4.034536, 1.5748);
  public static final Translation3d redHubPose = new Translation3d(11.938, 4.034536, 1.5748);
  public static final Pose3d blueHubPose3d = new Pose3d(blueHubPose, Rotation3d.kZero);
  public static final Pose3d redHubPose3d = new Pose3d(redHubPose, Rotation3d.kZero);

  public static final Translation3d blueRightCorner = new Translation3d(0.0, 0.0, 0.0);
  public static final Pose3d blueRightCornerPose3d = new Pose3d(blueRightCorner, Rotation3d.kZero);
  public static final Translation3d redRightCorner = new Translation3d(16.540988, 8.069326, 0.0);
  public static final Pose3d redRightCornerPose3d = new Pose3d(redRightCorner, Rotation3d.kZero);
  public static final Translation3d blueLeftCorner = new Translation3d(0, 8.069326, 0.0);
  public static final Pose3d blueLeftCornerPose3d = new Pose3d(blueLeftCorner, Rotation3d.kZero);
  public static final Translation3d redLeftCorner = new Translation3d(16.540988, 0, 0.0);
  public static final Pose3d redLeftCornerPose3d = new Pose3d(redLeftCorner, Rotation3d.kZero);
}
