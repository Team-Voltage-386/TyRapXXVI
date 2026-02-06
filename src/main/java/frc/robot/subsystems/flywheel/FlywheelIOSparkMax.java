package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.constants.jr.TurretConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.util.TuningUtil;

/**
 * Turret IO on a Spark MAX motor.
 * <p>
 * Also includes the shooter functionality for simplicity.
 */
public class FlywheelIOSparkMax implements FlywheelIO {

  private final SparkMax flywheelMotor =
      new SparkMax(TurretConstants.flywheelCanId, MotorType.kBrushless);
  // NEEDS the absolute encoder board plugged into MAX controller--not normal board.
  private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

  private SparkMaxConfig flywheelConfig;

  TuningUtil flywheelKp = new TuningUtil("/Tuning/turret/flywheelKp", .0);
  TuningUtil flywheelKd = new TuningUtil("/Tuning/turret/flywheelKd", 0.0);

  public FlywheelIOSparkMax() {
    flywheelConfig = new SparkMaxConfig();
    flywheelConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
    flywheelConfig
        .encoder
        .uvwAverageDepth(2)
        .positionConversionFactor(gearRatioPerRot)
        .velocityConversionFactor(gearRatioPerRot);
    flywheelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .pid(flywheelKp.getValue(), 0.0, flywheelKd.getValue());
    flywheelConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(300)
        .cruiseVelocity(600)
        .allowedProfileError(1000);
    flywheelConfig.closedLoop.feedForward.kV(turretKv).kS(turretKs);
    flywheelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        5,
        () ->
            flywheelMotor.configure(
                flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    flywheelEncoder.setPosition(0);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelKp
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated turret kp");
              flywheelConfig.closedLoop.pid(kp, 0.0, flywheelKp.getValue());
              flywheelMotor.configure(
                  flywheelConfig,
                  ResetMode.kNoResetSafeParameters,
                  PersistMode.kNoPersistParameters);
            });
    flywheelKd
        .get()
        .ifPresent(
            kd -> {
              System.out.println("updated turret kd");
              flywheelConfig.closedLoop.pid(flywheelKp.getValue(), 0.0, kd);
              flywheelMotor.configure(
                  flywheelConfig,
                  ResetMode.kNoResetSafeParameters,
                  PersistMode.kNoPersistParameters);
            });
    inputs.connected = true;
    inputs.flywheelSpeed = RPM.of(flywheelEncoder.getVelocity());
  }

  /** Set the Flywheel to the specific speed. */
  public void testFlywheelVoltage(double volts) {
    flywheelMotor.setVoltage(volts);
  }
}
