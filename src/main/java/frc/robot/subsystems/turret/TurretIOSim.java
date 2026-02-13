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
 */
public class TurretIOSim implements TurretIO, Simulatable {

  private Rotation2d turretYaw = new Rotation2d();
  private Rotation2d turretPitch = new Rotation2d();
  boolean flywheelShooting = true;

  private final Supplier<Pose2d> dtPose;
  private final Supplier<ChassisSpeeds> speedSupplier;
  private final SpindexerSubsystem spindexerSubsystem;
  private final Flywheel flywheel;
  private final IntakeIOSim intakeIOSim;

  public TurretIOSim(
      Supplier<Pose2d> pose3dSupplier,
      Supplier<ChassisSpeeds> speedSupplier,
      SpindexerSubsystem spindexer,
      Flywheel flywheel,
      IntakeIOSim intakeIOSim) {
    this.dtPose = pose3dSupplier;
    this.speedSupplier = speedSupplier;
    this.spindexerSubsystem = spindexer;
    this.flywheel = flywheel;
    this.intakeIOSim = intakeIOSim;
  }

  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected = true;
    inputs.turretYaw = turretYaw;
    inputs.turretPitch = turretPitch;
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    turretYaw = new Rotation2d(position.getRadians());
    Logger.recordOutput("Shooter/Turret/TurretYaw", turretYaw);
  }

  /* Set the turret pitch to the specified position. */
  @Override
  public void setTurretPitch(Rotation2d position) {
    turretPitch = new Rotation2d(MathUtil.clamp(position.getRadians(), 0, Math.PI / 2));
  }

  protected int tickCount = 0;

  @Override
  public void simulationSubTick(int i) {
    if (spindexerSubsystem.feederOn
        && i == 0
        && ++tickCount % 10 == 0
        && this.intakeIOSim.getBallCount() > 0) {
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
                          .plus(turretYaw),
                      // Initial height of the flying note
                      Meter.of(0.559),
                      // The launch speed is proportional to the RPM; assumed to be 16 meters/second
                      // at 6000
                      // RPM
                      MetersPerSecond.of(
                          flywheel.getFlywheelVelocity()
                              * TurretConstants.turretRPMToMetersPerSecond),
                      // The angle at which the note is launched
                      turretPitch.getMeasure())
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
                  .disableBecomesGamePieceOnFieldAfterTouchGround();
      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
      this.intakeIOSim.removeBall();
    }
  }
}
