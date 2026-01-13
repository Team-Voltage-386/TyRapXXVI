package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.jr.DriveConstants;
import lombok.Getter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.COTS.WHEELS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
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

  public SimContainer() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      throw new IllegalStateException("SimContainer can only be instantiated in SIM mode");
    }

    arena = new Arena2025Reefscape();

    driveSim =
        new SwerveDriveSimulation(
            driveTrainSimulationConfig, new Pose2d(8.790, 0.815, Rotation2d.kZero));
    arena.addDriveTrainSimulation(driveSim);
  }

  public void simulationInit(ResetOdo resetOdometry) {
    arena.resetFieldForAuto();
    resetOdometry.apply(driveSim.getSimulatedDriveTrainPose());
  }

  public void registerSimulator(Simulatable sim) {
    arena.addCustomSimulation(sim);
  }

  public void simulationPeriodic() {
    arena.simulationPeriodic();

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  @FunctionalInterface
  public interface ResetOdo {

    void apply(Pose2d pose);
  }
}
