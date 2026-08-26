package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.IntakeIOSim;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Physics sim implementation of turret IO.
 * <p>
 * Also includes the shooter functionality for simplicity.
 * This function has outdated comments, for use with crescendo bot. TODO: Update comments to rebuilt?
 */
public class TurretIOSim implements TurretIO, Simulatable {

  private Rotation2d turretYaw = new Rotation2d();
  private Rotation2d turretPitch = new Rotation2d();

  boolean flywheelShooting = false;

  private final Supplier<Pose2d> dtPose;
  private final Supplier<ChassisSpeeds> speedSupplier;
  private final double maxErrorAngleYaw = Math.toRadians(2); // In degrees, converted to radians
  private final double maxErrorAnglePitch = Math.toRadians(3); // In degrees, converted to radians.
  private final double maxErrorVelocity = 50; // In RPM

  /**
   * The launch speed is proportional to flywheel RPM, and is 16 m/s at 6000 RPM.
   *
   * <p>Derived from that statement rather than written as a literal, because the literal drifted
   * from it: this multiplied by 0.58, which is 14% high, and 14% high on the speed puts the fuel
   * over the HUB at every range in {@link Scoring}'s table -- 0.6 m above the goal at 1.6 m and
   * 1.25 m above it at 5.5 m, against a goal radius of 0.597 m. The shot was unscoreable past
   * about 4 m and marginal inside it.
   *
   * <p>The value it works out to, a little over half, is also what a single backed wheel does
   * physically: the contact point matches the wheel's surface speed, so the ball's centre leaves
   * at roughly half of it.
   */
  private static final double shotSpeedAt6000RpmMPS = 16.0;

  private static final double flywheelSurfaceSpeedToShotSpeed =
      shotSpeedAt6000RpmMPS / (6000.0 * TurretConstants.turretRPMToMetersPerSecond);
  private final IntakeIOSim intakeIOSim;
  private final SpindexerSubsystem spindexerSubsystem;
  private final Flywheel flywheel;

  public TurretIOSim(
      Supplier<Pose2d> pose3dSupplier,
      Supplier<ChassisSpeeds> speedSupplier,
      SpindexerSubsystem spindexer,
      Flywheel flywheel,
      IntakeIOSim intakeIOSim) {
    this.dtPose = pose3dSupplier;
    this.speedSupplier = speedSupplier;
    this.intakeIOSim = intakeIOSim;
    this.spindexerSubsystem = spindexer;
    this.flywheel = flywheel;
  }

  private double randomOffsetAngleYaw(boolean randomBothWays) {
    if (randomBothWays) {
      return (2 * Math.random() * maxErrorAngleYaw) - maxErrorAngleYaw;
    }
    return Math.random() * maxErrorAngleYaw;
  }

  private double randomOffsetAnglePitch(boolean randomBothWays) {
    if (randomBothWays) {
      return (2 * Math.random() * maxErrorAnglePitch) - maxErrorAnglePitch;
    }
    return Math.random() * maxErrorAnglePitch;
  }

  private double randomOffsetVelocity(boolean randomBothWays) {
    if (randomBothWays) {
      return (2 * Math.random() * maxErrorVelocity) - maxErrorVelocity;
    }
    return Math.random() * maxErrorVelocity;
  }

  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected = true;
    inputs.turretYaw = turretYaw;
    inputs.turretPitch = turretPitch;
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    turretYaw = new Rotation2d(MathUtil.clamp(position.getRadians(), -2 * Math.PI, 2 * Math.PI));
  }

  /* Set the turret pitch to the specified position. */
  @Override
  public void setTurretPitch(Rotation2d position) {
    turretPitch = new Rotation2d(MathUtil.clamp(position.getRadians(), 0, Math.PI / 2));
  }

  protected int tickCount = 0;
  private double calculatedVelocity;

  @Override
  public void simulationSubTick(int i) {
    if (spindexerSubsystem.feederOn
        && i == 0
        && ++tickCount % 10 == 0
        && this.intakeIOSim.getBallCount() > 0) {
      calculatedVelocity =
          (flywheel.getFlywheelVelocity() - randomOffsetVelocity(true))
              * TurretConstants.turretRPMToMetersPerSecond
              * flywheelSurfaceSpeedToShotSpeed;
      Logger.recordOutput("Simulation/Shooter/calculatedVelocity", calculatedVelocity);
      RebuiltFuelOnFly fuelOnFly =
          (RebuiltFuelOnFly)
              new RebuiltFuelOnFly(
                      // Specify the position of the chassis when the note is launched
                      dtPose.get().getTranslation(),
                      // Specify the translation of the shooter from the robot center (in the
                      // shooter’s
                      // reference frame)
                      TurretConstants.turretPosition,
                      // Specify the field-relative speed of the chassis, adding it to the initial
                      // velocity
                      // of the projectile
                      speedSupplier.get(),
                      // The shooter facing direction is the same as the robot’s facing direction
                      dtPose
                          .get()
                          .getRotation()
                          // Add the shooter’s rotation
                          .plus(turretYaw)
                          .plus(new Rotation2d(randomOffsetAngleYaw(true))),
                      // Initial height of the flying note
                      Meter.of(0.559),
                      // The launch speed, see flywheelSurfaceSpeedToShotSpeed
                      MetersPerSecond.of(calculatedVelocity),
                      // The angle at which the note is launched
                      turretPitch
                          .getMeasure()
                          .minus(new Rotation2d(randomOffsetAnglePitch(true)).getMeasure()))
                  // Set the target center to the Crescendo Speaker of the current alliance
                  .withTargetPosition(
                      () ->
                          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                              ? Constants.blueHubPose
                              : Constants.redHubPose)
                  // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
                  // speaker's "mouth")
                  .withTargetTolerance(new Translation3d(.5, .5, .5))
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
      this.intakeIOSim.removeBall();
    }
  }
}
