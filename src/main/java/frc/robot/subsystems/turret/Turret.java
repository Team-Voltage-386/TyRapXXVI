package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Handles turret control and aiming functionality */
public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Pose3d[] turretVisual = new Pose3d[2];

  private final Supplier<Pose2d> dtPose;

  public Turret(TurretIO io, Supplier<Pose2d> dtPose) {
    this.io = io;
    this.dtPose = dtPose;

    io.setTurretPitch(Rotation2d.fromDegrees(45));
    io.setTurretYaw(Rotation2d.k180deg);
  }

  public Command addPitchCommand(Rotation2d deltaPitch) {
    return runOnce(() -> io.setTurretPitch(inputs.turretPitch.plus(deltaPitch)));
  }

  public Command addYawCommand(Rotation2d deltaYaw) {
    return runOnce(() -> io.setTurretYaw(inputs.turretYaw.plus(deltaYaw)));
  }

  /**
   * Aims the turret to shoot a Fuel through the specified target pose.
   *
   * @param targetPose - The pose to hit with the Fuel.
   * @return The command to aim the turret.
   */
  // TODO: also calculate the optimal flywheel speed to hit the target at the same time.
  // TODO: ensure robot movement is included in the calculation.
  public Command aimAtCommand(Supplier<LinearVelocity> shootSpeed, Pose3d targetPose) {
    return runOnce(
        () -> {
          // deltaY = v0 sin(theta) * t - 0.5 g t^2
          Translation3d dtPos =
              new Translation3d(
                  dtPose.get().getTranslation().getX(), dtPose.get().getTranslation().getY(), 0.5);
          Translation3d deltaPos = targetPose.getTranslation().minus(dtPos);
          double g = 9.81; // m/s^2
          double v0 = shootSpeed.get().in(MetersPerSecond);
          double theta = calcHitPitch(deltaPos, v0, g);

          double yaw = Math.atan2(deltaPos.getY(), deltaPos.getX());
          io.setTurretPitch(new Rotation2d(theta));
          io.setTurretYaw(new Rotation2d(yaw));
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

    // Choose the higher angle to ensure it goes over the target
    double chosenAngle = Math.max(angle1, angle2);

    // Clamp the angle to [0, pi/2]
    return Math.max(0, Math.min(Math.PI / 2, chosenAngle));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret/Inputs", inputs);

    Translation3d dtPos =
        new Translation3d(
            dtPose.get().getTranslation().getX(), dtPose.get().getTranslation().getY(), 0.5);
    turretVisual[0] = new Pose3d(dtPos, new Rotation3d());
    turretVisual[1] =
        new Pose3d(
            dtPos.plus(
                new Translation3d(
                    inputs.turretYaw.getCos() * 0.5,
                    inputs.turretYaw.getSin() * 0.5,
                    inputs.turretPitch.getSin() * 0.65)),
            new Rotation3d());

    Logger.recordOutput("Shooter/Turret/Visual", turretVisual);
  }
}
