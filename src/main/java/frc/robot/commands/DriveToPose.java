package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.constants.jr.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private final Drive dt;
  private final PathPlannerTrajectoryState goalState;

  // Meters
  private static final double distanceTolerance = 0.1;
  // Radians
  private static final double angleTolerance = Math.toRadians(5);
  // Meters per second and radians per second
  private static final double speedTolerance = 0.1;

  private final PPHolonomicDriveController pid;

  private final Pose2d[] trajectory = new Pose2d[2];

  /**
   * Command to drive to a specific pose. Without a specified target speed, the command will end
   * with a stop at the target pose.
   * <p>
   * Does not use pathfinding; just drives in a straight line to the target. In the future, could be
   * improved with obstacle avoidance if we need longer distances.
   *
   * @param drivetrain The drive subsystem to use.
   * @param goalPose The target pose to drive to.
   */
  public DriveToPose(Drive drivetrain, Pose2d goalPose) {
    this(drivetrain, goalStateWithPose(goalPose));
  }

  // helper function to avoid duplication of constructor logic
  private static PathPlannerTrajectoryState goalStateWithPose(Pose2d goalPose) {
    PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
    state.pose = goalPose;
    return state;
  }

  /**
   * Command to drive to a specific pose.
   *
   * @param drivetrain The drive subsystem to use.
   * @param goalState The target state to drive to, including pose and speed.
   */
  public DriveToPose(Drive drivetrain, PathPlannerTrajectoryState goalState) {
    dt = drivetrain;
    this.goalState = goalState;

    pid = new PPHolonomicDriveController(DriveConstants.translationPID, DriveConstants.rotationPID);

    trajectory[1] = goalState.pose;

    addRequirements(dt);
  }

  public static Pose2d tagPose(int tagId, Transform2d tagOffset) {
    Pose2d tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId).orElseThrow().toPose2d();
    return new Pose2d(
        tagPose
            .getTranslation()
            .plus(
                new Translation2d(tagPose.getRotation().getCos(), tagPose.getRotation().getSin())
                    .times(.4)
                    .plus(tagOffset.getTranslation().rotateBy(tagPose.getRotation()))),
        tagPose.getRotation().rotateBy(Rotation2d.k180deg).rotateBy(tagOffset.getRotation()));
  }

  @Override
  public void initialize() {
    trajectory[0] = dt.getPose();
    Logger.recordOutput("Odometry/Trajectory", trajectory);
    Logger.recordOutput("Odometry/TrajectorySetpoint", goalState.pose);
    pid.reset(dt.getPose(), dt.getChassisSpeeds());
  }

  @Override
  public void execute() {
    dt.runVelocity(pid.calculateRobotRelativeSpeeds(dt.getPose(), goalState));
  }

  @Override
  public boolean isFinished() {
    boolean translationDone =
        Math.abs(dt.getPose().getTranslation().getDistance(goalState.pose.getTranslation()))
            < distanceTolerance;

    boolean rotationDone =
        MathUtil.isNear(
            dt.getRotation().getRadians(),
            goalState.pose.getRotation().getRadians(),
            angleTolerance);

    ChassisSpeeds speedDiff = dt.getChassisSpeeds().minus(goalState.fieldSpeeds);

    boolean speedDone =
        speedDiff.vxMetersPerSecond < speedTolerance
            && speedDiff.vyMetersPerSecond < speedTolerance
            && speedDiff.omegaRadiansPerSecond < speedTolerance;

    return translationDone && rotationDone && speedDone;
  }
}
