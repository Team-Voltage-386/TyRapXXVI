package frc.robot.subsystems.drive;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TuningUtil;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Max drive motor controller, Spark Max turn motor controller,
 * and CANcoder absolute encoder.
 * <p>
 * Does not support high-frequency encoding
 */
public class ModuleIOSparkMaxCancoder implements ModuleIO {

  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final CANcoder turnEncoder;

  private final TuningUtil driveKp =
      new TuningUtil("/Tuning/Drive/DriveKp", DriveConstants.driveKp);
  private final TuningUtil driveKd =
      new TuningUtil("/Tuning/Drive/DriveKd", DriveConstants.driveKd);

  private final TuningUtil driveKs =
      new TuningUtil("/Tuning/Drive/DriveKs", DriveConstants.driveKs);
  private final TuningUtil driveKv =
      new TuningUtil("/Tuning/Drive/DriveKv", DriveConstants.driveKv);

  private final TuningUtil turnKp = new TuningUtil("/Tuning/Drive/TurnKp", DriveConstants.turnKp);
  private final TuningUtil turnKd = new TuningUtil("/Tuning/Drive/TurnKd", DriveConstants.turnKd);

  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final PIDController turnController;

  private boolean turnClosedLoop = true;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private SparkMaxConfig driveConfig;
  private SparkMaxConfig turnConfig;

  public ModuleIOSparkMaxCancoder(int module) {
    zeroRotation = DriveConstants.zeroRotations[module];
    driveSpark = new SparkMax(DriveConstants.driveCanIds[module], MotorType.kBrushless);
    turnSpark = new SparkMax(DriveConstants.turnCanIds[module], MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();

    // Configure drive motor
    driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.driveCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(DriveConstants.driveKp, 0.0, DriveConstants.driveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(DriveConstants.turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.turnCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // configure turn CANcoder and PID controller
    turnController = new PIDController(DriveConstants.turnKp, 0, DriveConstants.turnKd);
    // Treat the turn angle as continuous (wraps at 0/2pi) so the controller takes the
    // shortest path around the circle. Also set a small tolerance and stop driving
    // the motor when within tolerance to avoid oscillation.
    turnController.enableContinuousInput(
        DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput);
    turnEncoder = new CANcoder(DriveConstants.turnCancoderIds[module]);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    // TODO: ensure this is right.
    turnEncoderConfig.MagnetSensor.SensorDirection =
        DriveConstants.turnEncoderInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(turnEncoderConfig, 0.25));

    turnPosition = turnEncoder.getPosition();
    turnVelocity = turnEncoder.getVelocity();

    turnPosition.setUpdateFrequency(50.0);
    turnVelocity.setUpdateFrequency(50.0);

    turnEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnPosition, turnVelocity);

    // update PID values if they have changed while tuning
    driveKp
        .get()
        .ifPresent(
            value -> {
              driveConfig.closedLoop.pid(value, 0.0, driveKd.getValue());
              driveSpark.configure(
                  driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    driveKd
        .get()
        .ifPresent(
            value -> {
              driveConfig.closedLoop.pid(driveKp.getValue(), 0.0, value);
              driveSpark.configure(
                  driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    turnKp.get().ifPresent(value -> turnController.setP(value));
    turnKd.get().ifPresent(value -> turnController.setD(value));

    double turnRads = turnPosition.getValueAsDouble() * DriveConstants.turnEncoderPositionFactor;
    if (turnClosedLoop) {
      double turnPos =
          MathUtil.inputModulus(
              turnRads, DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput);
      double volts = turnController.calculate(turnPos);
      // If we're within tolerance, don't drive the motor to avoid hunting.
      if (turnController.atSetpoint()) {
        volts = 0.0;
      }
      // Clamp to safe voltage range
      volts = MathUtil.clamp(volts, -12.0, 12.0);
      turnSpark.setVoltage(volts);
    }

    // Update turn inputs
    sparkStickyFault = false;
    inputs.turnPosition = new Rotation2d(turnRads).minus(zeroRotation);
    inputs.turnVelocityRadPerSec =
        turnVelocity.getValueAsDouble() * DriveConstants.turnEncoderVelocityFactor;
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        driveKs.getValue() * Math.signum(velocityRadPerSec)
            + driveKv.getValue() * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(),
            DriveConstants.turnPIDMinInput,
            DriveConstants.turnPIDMaxInput);
    turnController.setSetpoint(setpoint);
  }
}
