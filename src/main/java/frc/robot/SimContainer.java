package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.COTS.WHEELS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class SimContainer {

  @Getter protected final SwerveDriveSimulation driveSim;

  private static final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(DriveConstants.robotMassKg))
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify swerve module (for realistic swerve dynamics)

          .withSwerveModule(
              COTS.ofMark4i(
                  DriveConstants.driveGearbox,
                  DriveConstants.turnGearbox,
                  WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  3))
          // Configures the track length and track width (spacing between swerve modules)
          .withCustomModuleTranslations(DriveConstants.moduleTranslations)
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(Inches.of(30), Inches.of(30));

  protected final SimulatedArena arena;

  /**
   * The other robots on the field, if anything outside asks for any.
   *
   * <p>Constructed unconditionally and not behind the bridge profile, because it does nothing at
   * all until a roster is published: a plain simulation session is four idle NT subscriptions
   * different from before. Exposed so that a caller can see how many robots actually got made,
   * which is the only handshake the far side gets.
   */
  @Getter protected final BridgeRobots bridgeRobots;

  public SimContainer() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      throw new IllegalStateException("SimContainer can only be instantiated in SIM mode");
    }

    arena = SimulatedArena.getInstance();

    driveSim =
        new SwerveDriveSimulation(
            driveTrainSimulationConfig, new Pose2d(8.790, 0.815, Rotation2d.kZero));
    arena.addDriveTrainSimulation(driveSim);

    bridgeRobots = new BridgeRobots(arena);
  }

  public void simulationInit(ResetOdo resetOdometry) {
    arena.resetFieldForAuto();
    resetOdometry.apply(driveSim.getSimulatedDriveTrainPose());
  }

  public void registerSimulator(Simulatable sim) {
    arena.addCustomSimulation(sim);
  }

  public void simulationPeriodic(VisionConsumer visionConsumer) {
    // Before the step, not after: these are setpoints for the tick that is
    // about to happen. Commanding afterwards means every extra robot acts on
    // a world one tick stale, which is a lag nobody would think to look for.
    bridgeRobots.periodic();

    arena.simulationPeriodic();

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

    visionConsumer.accept(
        driveSim.getSimulatedDriveTrainPose(),
        Timer.getTimestamp(),
        VecBuilder.fill(0.0, 0.0, 0.0));
  }

  @FunctionalInterface
  public interface ResetOdo {

    void apply(Pose2d pose);
  }
}
