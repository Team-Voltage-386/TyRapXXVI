package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

/**
 * Physics sim implementation of turret IO.
 * <p>
 * Also includes the shooter functionality for simplicity.
 */
public class TurretIOSim implements TurretIO, Simulatable {

  private Rotation2d turretYaw = new Rotation2d();
  private Rotation2d turretPitch = new Rotation2d();

  private double flywheelRPM = 0.0;

  boolean flywheelShooting = false;

  private final Supplier<Pose2d> dtPose;
  private final Supplier<ChassisSpeeds> speedSupplier;

  public TurretIOSim(Supplier<Pose2d> pose3dSupplier, Supplier<ChassisSpeeds> speedSupplier) {
    this.dtPose = pose3dSupplier;
    this.speedSupplier = speedSupplier;
  }

  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected = true;
    inputs.turretYaw = turretYaw;
    inputs.turretPitch = turretPitch;
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    turretYaw = new Rotation2d(MathUtil.clamp(position.getRadians(), -Math.PI, Math.PI));
  }

  /* Set the turret pitch to the specified position. */
  @Override
  public void setTurretPitch(Rotation2d position) {
    turretPitch = new Rotation2d(MathUtil.clamp(position.getRadians(), 0, Math.PI / 2));
  }

  public void setFlywheelShooting(boolean shooting) {
    this.flywheelShooting = shooting;
  }

  public void setFlywheelSpeed(AngularVelocity speed) {
    this.flywheelRPM = speed.in(RPM);
  }

  @Override
  public void simulationSubTick(int i) {
    if (flywheelShooting && i % 4 == 0) {
      RebuiltFuelOnFly fuelOnFly =
          (RebuiltFuelOnFly)
              new RebuiltFuelOnFly(
                      // Specify the position of the chassis when the note is launched
                      dtPose.get().getTranslation(),
                      // Specify the translation of the shooter from the robot center (in the
                      // shooter’s
                      // reference frame)
                      new Translation2d(0.0, 0),
                      // Specify the field-relative speed of the chassis, adding it to the initial
                      // velocity
                      // of the projectile
                      speedSupplier.get(),
                      // The shooter facing direction is the same as the robot’s facing direction
                      dtPose
                          .get()
                          .getRotation()
                          // Add the shooter’s rotation
                          .plus(turretYaw),
                      // Initial height of the flying note
                      Meter.of(0.5),
                      // The launch speed is proportional to the RPM; assumed to be 16 meters/second
                      // at 6000
                      // RPM
                      MetersPerSecond.of(flywheelRPM / 6000 * 12),
                      // The angle at which the note is launched
                      turretPitch.getMeasure())
                  // Set the target center to the Crescendo Speaker of the current alliance
                  .withTargetPosition(
                      () ->
                          FieldMirroringUtils.toCurrentAllianceTranslation(
                              new Translation3d(11.9, 4.1, 1.5)))
                  // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
                  // speaker's "mouth")
                  .withTargetTolerance(new Translation3d(1, 1, 1))
                  // Set a callback to run when the note hits the target
                  .withHitTargetCallBack(() -> System.out.println("Hit speaker, +2 points!"))
                  // Configure callbacks to visualize the flight trajectory of the projectile
                  .withProjectileTrajectoryDisplayCallBack(
                      // Callback for when the note will eventually hit the target (if configured)
                      (pose3ds) ->
                          Logger.recordOutput(
                              "Shooter/Simulation/FuelProjectileSuccessfulShot",
                              pose3ds.toArray(Pose3d[]::new)),
                      // Callback for when the note will eventually miss the target, or if no target
                      // is configured
                      (pose3ds) ->
                          Logger.recordOutput(
                              "Shooter/Simulation/FuelProjectileUnsuccessfulShot",
                              pose3ds.toArray(Pose3d[]::new)))
                  .enableBecomesGamePieceOnFieldAfterTouchGround();
      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }
  }
}
