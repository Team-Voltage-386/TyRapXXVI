package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.sim.DriveConstants;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController;
  private final PIDController turnController;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final SwerveModuleSimulation moduleSim;

  private final GenericMotorController driveMotor;
  private final GenericMotorController turnMotor;

  public ModuleIOSim(SwerveModuleSimulation moduleSim) {
    this.moduleSim = moduleSim;

    driveController = new PIDController(DriveConstants.driveKp, 0, DriveConstants.driveKd);
    turnController = new PIDController(DriveConstants.turnKp, 0, DriveConstants.turnKd);

    // configures a generic motor controller for drive motor
    // set a current limit of 60 amps
    this.driveMotor =
        moduleSim
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(DriveConstants.driveCurrentLimit));
    this.turnMotor =
        moduleSim
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(DriveConstants.turnCurrentLimit));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(moduleSim.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    driveMotor.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));
    turnMotor.requestVoltage(Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSim.getDriveMotorStatorCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = moduleSim.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSim.getSteerMotorStatorCurrent().in(Amps));

    // Update odometry inputs
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts =
        DriveConstants.driveKs * Math.signum(velocityRadPerSec)
            + DriveConstants.driveKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
