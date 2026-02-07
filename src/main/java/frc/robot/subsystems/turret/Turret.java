package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Handles turret control and aiming functionality */
public class Turret extends SubsystemBase {

  public final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Pose3d[] turretVisual = new Pose3d[2];

  private final Supplier<Pose2d> dtPose;
  private Pose3d currentTargetPose = new Pose3d();
  private final Flywheel flywheel;

  private boolean autoAimEnabled = false;

  public Turret(TurretIO io, Supplier<Pose2d> dtPose, Flywheel flywheel) {
    this.io = io;
    this.dtPose = dtPose;
    this.flywheel = flywheel;

    io.setTurretPitch(Rotation2d.fromDegrees(45));
    io.setTurretYaw(Rotation2d.kZero);
  }

  public Command addPitchCommand(Rotation2d deltaPitch) {
    return runOnce(() -> io.setTurretPitch(inputs.turretPitch.plus(deltaPitch)));
  }

  public Command addYawCommand(Rotation2d deltaYaw) {
    return runOnce(() -> io.setTurretYaw(inputs.turretYaw.plus(deltaYaw)));
  }

  public Command enableAutoAimCommand(Pose3d targetPose) {
    return runOnce(
        () -> {
          autoAimEnabled = true;
          currentTargetPose = targetPose;
        });
  }

  public Command disableAutoAimCommand() {
    return runOnce(() -> autoAimEnabled = false);
  }

  public LinearVelocity calculateShotSpeed(Pose3d targetPose) {
    // Placholder: Need real calculations
    return MetersPerSecond.of(9.0);
  }

  public void aimAtTarget(Pose3d targetPose) {
    // deltaY = v0 sin(theta) * t - 0.5 g t^2
    LinearVelocity shootSpeed = calculateShotSpeed(targetPose);
    Pose2d dtPos = dtPose.get();
    Pose2d turretFieldPos =
        dtPos.plus(
            new Transform2d(
                TurretConstants.turretPosition.rotateBy(dtPos.getRotation()), Rotation2d.kZero));
    Translation3d turretFieldTrans =
        new Translation3d(
            turretFieldPos.getTranslation().getX(), turretFieldPos.getTranslation().getY(), 0.5);
    Translation3d deltaPos = targetPose.getTranslation().minus(turretFieldTrans);
    // TODO: ensure this g constant is accurate for real-world
    double g = 11; // m/s^2
    double v0 = shootSpeed.in(MetersPerSecond);
    double theta = calcHitPitch(deltaPos, v0, g);

    double yaw = Math.atan2(deltaPos.getY(), deltaPos.getX());
    io.setTurretPitch(new Rotation2d(theta));
    io.setTurretYaw(new Rotation2d(yaw).minus(dtPos.getRotation()));

    double shooterWheelRPM = (v0 / (2 * Math.PI * TurretConstants.shooterWheelRadiusMeters)) * 60;
    flywheel.setFlywheelSpeed(shooterWheelRPM);
    Logger.recordOutput("Shooter/Turret/ShooterWheelRPM", shooterWheelRPM);
  }

  /**
   * Aims the turret to shoot a Fuel through the specified target pose.
   *
   * @param targetPose - The pose to hit with the Fuel.
   * @return The command to aim the turret.
   */
  // TODO: calculate the optimal constant flywheel speed
  // TODO: ensure robot movement is included in the calculation.
  public Command aimAtCommand(Supplier<LinearVelocity> shootSpeed, Pose3d targetPose) {
    return runOnce(
        () -> {
          aimAtTarget(targetPose);
        });
  }

  // Calculate hit pitch with a range of [0, pi/2] radians.
  // this also accounts for the robot being on either side of the target.
  private static double calcHitPitch(Translation3d deltaPos, double v0, double g) {
    double x = Math.hypot(deltaPos.getX(), deltaPos.getY());
    double y = deltaPos.getZ();

    double v0Squared = v0 * v0;
    double underSqrt = v0Squared * v0Squared - g * (g * x * x + 2 * y * v0Squared);

    if (underSqrt < 0) {
      // No valid solution, return 45 degrees as a default
      return Math.PI / 4;
    }

    double sqrtPart = Math.sqrt(underSqrt);
    double angle1 = Math.atan((v0Squared + sqrtPart) / (g * x));
    double angle2 = Math.atan((v0Squared - sqrtPart) / (g * x));

    // Choose the higher angle to ensure it goes over the walls
    double chosenAngle = Math.max(angle1, angle2);

    // Clamp the angle to [0, pi/2]
    return Math.max(0, Math.min(Math.PI / 2, chosenAngle));
  }

  @Override
  public void periodic() {

    if (autoAimEnabled) {
      aimAtTarget(currentTargetPose);
    } else {
      flywheel.setFlywheelSpeed(0);
    }

    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret/Inputs", inputs);

    Pose2d pose =
        dtPose.get().plus(new Transform2d(TurretConstants.turretPosition, Rotation2d.kZero));
    Translation3d dtPos3d =
        new Translation3d(pose.getTranslation().getX(), pose.getTranslation().getY(), 0.5);
    turretVisual[0] = new Pose3d(dtPos3d, new Rotation3d());
    turretVisual[1] =
        new Pose3d(
            dtPos3d.plus(
                new Translation3d(
                    inputs.turretYaw.plus(pose.getRotation()).getCos() * 0.5,
                    inputs.turretYaw.plus(pose.getRotation()).getSin() * 0.5,
                    // TODO: this isn't accurate for high pitch angles
                    inputs.turretPitch.getSin() * 0.65)),
            new Rotation3d(dtPose.get().getRotation()));

    Logger.recordOutput("Shooter/Turret/Visual", turretVisual);
  }
}
