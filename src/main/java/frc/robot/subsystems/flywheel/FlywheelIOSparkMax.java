package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.util.TuningUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Turret IO on a Spark MAX motor.
 * <p>
 * Also includes the shooter functionality for simplicity.
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  // real robot will likely have two motors, use follower
  private final SparkMax flywheelMotorMaster =
      new SparkMax(TurretConstants.flywheelMasterCanId, MotorType.kBrushless);
  private final SparkMax flywheelMotorSlave =
      new SparkMax(TurretConstants.flywheelSlaveCanId, MotorType.kBrushless);
  private final RelativeEncoder flywheelEncoder = flywheelMotorMaster.getEncoder();
  private double flywheelSetpoint;

  private double outputVoltage;

  private SparkMaxConfig flywheelConfig;

  TuningUtil flywheelKp = new TuningUtil("/Tuning/flywheel/flywheelKp", 0.001);
  TuningUtil flywheelKd = new TuningUtil("/Tuning/flywheel/flywheelKd", 0.02);
  TuningUtil flywheelKv = new TuningUtil("/Tuning/flywheel/flywheelKv", TurretConstants.flywheelKv);
  TuningUtil flywheelKs = new TuningUtil("/Tuning/flywheel/flywheelKs", TurretConstants.flywheelKs);
  TuningUtil flywheelKa = new TuningUtil("/Tuning/flywheel/flywheelKa", TurretConstants.flywheelKa);
  TuningUtil threshold = new TuningUtil("/Tuning/flywheel/Threshold", 400);
  TuningUtil rateLimit = new TuningUtil("/Tuning/flywheel/RateLimit", 200);
  SlewRateLimiter filter = new SlewRateLimiter(rateLimit.getValue());

  private PIDController flywheelPID =
      new PIDController(flywheelKp.getValue(), 0.0, flywheelKd.getValue());
  private SimpleMotorFeedforward flywheelFeedforward =
      new SimpleMotorFeedforward(
          flywheelKs.getValue(), flywheelKv.getValue(), flywheelKa.getValue());

  public FlywheelIOSparkMax() {
    flywheelConfig = new SparkMaxConfig();
    flywheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12.0);
    flywheelConfig.encoder.uvwAverageDepth(4).uvwMeasurementPeriod(16);
    flywheelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .pid(flywheelKp.getValue(), 0.0, flywheelKd.getValue(), ClosedLoopSlot.kSlot1);
    flywheelConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(400)
        .cruiseVelocity(600)
        .allowedProfileError(1000);
    flywheelConfig
        .closedLoop
        .feedForward
        .kV(flywheelKv.getValue(), ClosedLoopSlot.kSlot0)
        .kS(flywheelKs.getValue(), ClosedLoopSlot.kSlot0)
        .kA(flywheelKa.getValue(), ClosedLoopSlot.kSlot0)
        .kV(flywheelKv.getValue(), ClosedLoopSlot.kSlot1)
        .kS(flywheelKs.getValue(), ClosedLoopSlot.kSlot1)
        .kA(flywheelKa.getValue(), ClosedLoopSlot.kSlot1);
    flywheelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(false)
        .primaryEncoderPositionPeriodMs(100)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        5,
        () ->
            flywheelMotorMaster.configure(
                flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        5,
        () ->
            flywheelMotorSlave.configure(
                flywheelConfig.follow(flywheelMotorMaster),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    flywheelEncoder.setPosition(0);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelKp
        .get()
        .ifPresent(
            kp -> {
              flywheelPID.setP(kp);
            });
    flywheelKd
        .get()
        .ifPresent(
            kd -> {
              flywheelPID.setD(kd);
            });
    flywheelKa
        .get()
        .ifPresent(
            ka -> {
              flywheelFeedforward =
                  new SimpleMotorFeedforward(flywheelKs.getValue(), flywheelKv.getValue(), ka);
            });
    flywheelKs
        .get()
        .ifPresent(
            ks -> {
              flywheelFeedforward =
                  new SimpleMotorFeedforward(ks, flywheelKv.getValue(), flywheelKa.getValue());
            });
    flywheelKv
        .get()
        .ifPresent(
            kv -> {
              flywheelFeedforward =
                  new SimpleMotorFeedforward(flywheelKs.getValue(), kv, flywheelKa.getValue());
            });

    inputs.connected = true;
    inputs.flywheelSpeed = RPM.of(flywheelEncoder.getVelocity());
    double velocity = flywheelEncoder.getVelocity();
    Logger.recordOutput(
        "/Shooter/Flywheel/VelocitySetpoint",
        flywheelMotorMaster.getClosedLoopController().getSetpoint());
    Logger.recordOutput(
        "/Shooter/Flywheel/MasterAppliedOutput",
        flywheelMotorMaster.getAppliedOutput() * flywheelMotorMaster.getBusVoltage());
    Logger.recordOutput(
        "/Shooter/Flywheel/SlaveAppliedOutput",
        flywheelMotorSlave.getAppliedOutput() * flywheelMotorMaster.getBusVoltage());
    Logger.recordOutput("/Shooter/Flywheel/Velocity", velocity);
    Logger.recordOutput("/Shooter/Flywheel/Current", flywheelMotorMaster.getOutputCurrent());
    readjustPID();
  }

  public void setFlywheelVelocity(double velocityRPM) {
    flywheelSetpoint = velocityRPM;

    Logger.recordOutput("/Shooter/Flywheel/VelocitySetpoint2", velocityRPM);
  }

  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }

  // to help the kp value from freaking out at low speeds
  public void readjustPID() {
    outputVoltage =
        flywheelFeedforward.calculate(flywheelSetpoint)
            + flywheelPID.calculate(flywheelEncoder.getVelocity(), flywheelSetpoint);
    flywheelMotorMaster.setVoltage(outputVoltage);
  }

  /** Set the Flywheel to the specific speed. */
  public void testFlywheelVoltage(double volts) {
    flywheelMotorMaster.setVoltage(volts);
  }
}
