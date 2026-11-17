package frc.robot.subsystems.turret;

import static frc.robot.constants.jr.TurretConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.jr.DriveConstants;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.util.TuningUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Turret IO on a Spark MAX motor.
 * <p>
 * Also includes the shooter functionality for simplicity.
 */
public class TurretIOSparkMax2 implements TurretIO {

  private final SparkMax yawMotor =
      new SparkMax(TurretConstants.turretYawCanId, MotorType.kBrushless);
  private final SparkMax hoodMotor =
      new SparkMax(TurretConstants.turretPitchCanId, MotorType.kBrushless);

  private final RelativeEncoder yawEncoder = yawMotor.getEncoder();
  private final SparkMaxConfig yawConfig;
  private final SparkMaxConfig hoodConfig;

  // Turret Limit Input
  DigitalInput turretLimitInput = new DigitalInput(9);
  Debouncer debouncer = new Debouncer(0.05, Debouncer.DebounceType.kFalling);

  TuningUtil yawKp = new TuningUtil("/Tuning/turret/yawKp", TurretConstants.turretYawKp);
  TuningUtil yawKd = new TuningUtil("/Tuning/turret/yawKd", TurretConstants.turretYawKd);
  TuningUtil yawKs = new TuningUtil("Tuning/turret/yawKs", TurretConstants.turretKs);
  TuningUtil yawKv = new TuningUtil("Tuning/turret/yawKv", TurretConstants.turretKv);
  TuningUtil hoodKp = new TuningUtil("/Tuning/turret/hoodKp", TurretConstants.hoodKP);
  TuningUtil hoodKd = new TuningUtil("/Tuning/turret/hoodKd", 0.0);
  TuningUtil turretRightLimit = new TuningUtil("/Tuning/turret/rightLimit", 0);
  TuningUtil turretLeftLimit = new TuningUtil("/Tuning/turret/leftLimit", 0);
  protected boolean clockwiseLimitHit = false;
  protected boolean counterclockwiseLimitHit = false;
  protected double desiredAngle = TurretConstants.turretStartupAngleDeg / 360.0;
  protected boolean manualMode = true;
  private double manualYawVolts = 0.0;
  private boolean yawCalibrated = false;

  public enum WhichLimit {
    LEFT,
    RIGHT,
    NULL
  }

  WhichLimit limitSwitchTriggered = WhichLimit.NULL;

  protected ProfiledPIDController yawController =
      new ProfiledPIDController(
          yawKp.getValue(),
          0.0,
          yawKd.getValue(),
          new TrapezoidProfile.Constraints(
              TurretConstants.turretMaxRotationSpeedRotPerSec,
              TurretConstants.turretMaxAccelRotPerSec2));
  protected SimpleMotorFeedforward yawFeedforward =
      new SimpleMotorFeedforward(0.0, TurretConstants.turretKv);

  protected double lastYawSpeed = 0.0;

  public TurretIOSparkMax2() {
    yawConfig = new SparkMaxConfig();
    yawConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
    yawConfig
        .encoder
        .uvwAverageDepth(2)
        .positionConversionFactor(gearRatioPerRot)
        .velocityConversionFactor(gearRatioPerRot);
    yawConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0, 0.0, 0.00);
    yawConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    yawConfig.inverted(true);
    yawConfig
        .softLimit
        .reverseSoftLimit(TurretConstants.turretMinAngleRot)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(TurretConstants.turretMaxAngleRot)
        .forwardSoftLimitEnabled(true);

    tryUntilOk(
        5,
        () ->
            yawMotor.configure(
                yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    calibrateYawAtStartup();

    hoodConfig = new SparkMaxConfig();
    hoodConfig.encoder.uvwAverageDepth(4).positionConversionFactor(1).velocityConversionFactor(1);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(hoodKp.getValue(), 0.0, 0.0);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12.0);
    hoodConfig.softLimit.forwardSoftLimit(0.74);
    tryUntilOk(
        5,
        () ->
            hoodMotor.configure(
                hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    hoodMotor.getEncoder().setPosition(0);
  }

  public void updateInputs(TurretIOInputs inputs) {
    yawKp
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated turret kp");
              yawController.setP(kp);
            });
    yawKd
        .get()
        .ifPresent(
            kd -> {
              System.out.println("updated turret kd");
              yawController.setD(kd);
            });
    yawKs
        .get()
        .ifPresent(
            ks -> {
              System.out.println("updated turret ks");
              yawFeedforward.setKs(ks);
            });
    yawKv
        .get()
        .ifPresent(
            kv -> {
              System.out.println("updated turret kv");
              yawFeedforward.setKv(kv);
            });
    hoodKp
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated hood kp");
              hoodConfig.closedLoop.pid(kp, 0.0, hoodKd.getValue());
              hoodMotor.configure(
                  hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    hoodKd
        .get()
        .ifPresent(
            kd -> {
              System.out.println("updated hood kd");
              hoodConfig.closedLoop.pid(hoodKp.getValue(), 0.0, kd);
              hoodMotor.configure(
                  hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });

    inputs.connected = true;
    Rotation2d yawTurretCenter = Rotation2d.fromRotations(yawEncoder.getPosition());

    Logger.recordOutput("/Shooter/Turret/ActualYawTurretCtr", yawTurretCenter);
    Logger.recordOutput("/Shooter/Turret/Encoder", yawEncoder.getPosition());
    inputs.turretYaw = yawTurretCenter.minus(TurretConstants.turretCenterOffsetRot);
    inputs.turretPitch = Rotation2d.fromRotations(hoodMotor.getEncoder().getPosition());
    inputs.turretLimitTrue = !turretLimitInput.get();
    double velocity = yawEncoder.getVelocity();
    double setpoint = yawMotor.getClosedLoopController().getSetpoint();
    double hoodsetpoint = hoodMotor.getClosedLoopController().getSetpoint();
    handleLimits();
    if (manualMode) {
      // Recheck every cycle, including a manual command that remains active at an endpoint.
      applyYawVoltage(manualYawVolts);
    } else {
      if (Math.abs(desiredAngle - yawEncoder.getPosition()) < (0.5 / 360.0)) {
        applyYawVoltage(0);
        yawController.reset(yawEncoder.getPosition());
        lastYawSpeed = 0.0;
      } else {
        double pidVal = yawController.calculate(yawEncoder.getPosition(), desiredAngle);
        double feedforwardVal =
            yawFeedforward.calculateWithVelocities(
                lastYawSpeed, yawController.getSetpoint().velocity);
        double outVoltage =
            MathUtil.clamp(
                pidVal + feedforwardVal,
                -TurretConstants.maxYawVoltage,
                TurretConstants.maxYawVoltage);
        if (outVoltage > 0) {
          outVoltage += TurretConstants.turretKs;
        } else if (outVoltage < 0) {
          outVoltage -= TurretConstants.turretKs;
        }

        Logger.recordOutput("Shooter/Turret/pidVal", pidVal);
        Logger.recordOutput("Shooter/Turret/feedForwardVal", feedforwardVal);
        Logger.recordOutput("Shooter/Turret/outVoltage", outVoltage);
        Logger.recordOutput(
            "Shooter/Turret/profiledSetpoint", yawController.getSetpoint().position);
        Logger.recordOutput(
            "Shooter/Turret/profiledVelSetpoint", yawController.getSetpoint().velocity);
        applyYawVoltage(outVoltage);
        lastYawSpeed = yawController.getSetpoint().velocity;
      }
    }

    Logger.recordOutput("/Shooter/Turret/Setpoint", setpoint);
    Logger.recordOutput("/Shooter/Turret/AppliedOutput", yawMotor.getAppliedOutput());
    Logger.recordOutput("/Shooter/Turret/Velocity", velocity);
    Logger.recordOutput("/Shooter/Turret/Current", yawMotor.getOutputCurrent());
    Logger.recordOutput("/Shooter/Turret/LimitSwitchTrue", inputs.turretLimitTrue);
    Logger.recordOutput("/Shooter/Turret/LimitSwitchTriggeredDirection", getLimitSwitch());
    Logger.recordOutput("/Shooter/Hood/AppliedOutput", hoodMotor.getAppliedOutput());
    Logger.recordOutput("/Shooter/Hood/Setpoint", hoodsetpoint);
    Logger.recordOutput("/Shooter/Hood/Current", hoodMotor.getOutputCurrent());
    Logger.recordOutput("/Shooter/Hood/position", hoodMotor.getEncoder().getPosition());
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    if (manualMode) {
      yawController.reset(yawEncoder.getPosition(), yawEncoder.getVelocity() / 60.0);
      lastYawSpeed = yawEncoder.getVelocity() / 60.0;
    }
    manualMode = false;
    Logger.recordOutput("/Shooter/Turret/DesiredAngleRobotCtr", position);
    double wrappedPositionDeg = TurretYawLimits.limitDegrees(position.getDegrees());
    Logger.recordOutput("/Shooter/Turret/DesiredAngleWrapped", wrappedPositionDeg);
    Logger.recordOutput(
        "/Shooter/Turret/DesiredAngleWrappedRot",
        Rotation2d.fromDegrees(wrappedPositionDeg).getRotations());
    // Keep this as an unwrapped scalar: 270 -> 20 travels through the permitted interval.
    desiredAngle = wrappedPositionDeg / 360.0;

    Logger.recordOutput(
        "/Shooter/Turret/DesiredAngleTurretCtr", Rotation2d.fromRotations(desiredAngle));
    /*yawMotor
    .getClosedLoopController()
    .setSetpoint(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);*/
  }

  public void testTurretVoltage(double volts) {
    manualMode = true;
    manualYawVolts = volts;
    applyYawVoltage(manualYawVolts);
  }

  private void applyYawVoltage(double volts) {
    double limitedVolts =
        yawCalibrated
            ? TurretYawLimits.limitVoltage(
                yawEncoder.getPosition(),
                volts,
                limitSwitchTriggered == WhichLimit.RIGHT,
                limitSwitchTriggered == WhichLimit.LEFT)
            : 0.0;
    yawMotor.setVoltage(limitedVolts);
    Logger.recordOutput("Shooter/Turret/limitedVoltage", limitedVolts);
  }

  public void resyncTurret() {
    testTurretVoltage(0);
    DriverStation.reportWarning(
        "Automatic turret homing is disabled until the limit-switch angle is measured for the "
            + "new chain. Place the turret at +20 degrees and use driver Start to calibrate.",
        false);
  }

  public void testHoodVoltage(double volts) {
    hoodMotor.setVoltage(volts);
  }

  /* Set the turret pitch to the specified position. */
  @Override
  public void setTurretPitch(Rotation2d position) {
    double desiredSetpoint =
        (TurretConstants.maxHoodSetpoint / TurretConstants.turretHoodAngleRange)
            * (TurretConstants.turretMaxHoodAngle - position.getDegrees());
    Logger.recordOutput("/Shooter/Hood/TrueDesiredAngle", position.getDegrees());
    Logger.recordOutput("/Shooter/Hood/CalculatedDesiredAngle", desiredSetpoint);
    hoodMotor.getClosedLoopController().setSetpoint(-desiredSetpoint, ControlType.kPosition);
    // No pitch control implemented
  }

  /** Assign +20 degrees to the current position; the turret must be physically at its startup mark. */
  public void calibrateYawAtStartup() {
    testTurretVoltage(0);
    yawCalibrated = false;
    double startupRot = TurretConstants.turretStartupAngleDeg / 360.0;
    tryUntilOk(
        5,
        () -> {
          REVLibError error = yawEncoder.setPosition(startupRot);
          yawCalibrated = error == REVLibError.kOk;
          return error;
        });
    desiredAngle = startupRot;
    yawController.reset(startupRot);
    lastYawSpeed = 0.0;
    limitSwitchTriggered = WhichLimit.NULL;
    if (!yawCalibrated) {
      DriverStation.reportError("Turret yaw calibration failed; yaw output is disabled.", false);
    }
    Logger.recordOutput("Shooter/Turret/yawCalibrated", yawCalibrated);
  }

  public void handleLimits() {
    if (debouncer.calculate(!turretLimitInput.get())) {
      // check velocity directions
      if (yawEncoder.getVelocity() > 0 && limitSwitchTriggered == WhichLimit.NULL) {
        limitSwitchTriggered = WhichLimit.LEFT;
      }
      if (yawEncoder.getVelocity() < 0 && limitSwitchTriggered == WhichLimit.NULL) {
        limitSwitchTriggered = WhichLimit.RIGHT;
      }
      if (limitSwitchTriggered == WhichLimit.NULL) {
        // A switch may already be active at startup, with no velocity to identify the endpoint.
        limitSwitchTriggered =
            yawEncoder.getPosition()
                    <= (TurretConstants.turretMinAngleRot + TurretConstants.turretMaxAngleRot) / 2.0
                ? WhichLimit.RIGHT
                : WhichLimit.LEFT;
      }
    } else {
      limitSwitchTriggered = WhichLimit.NULL;
    }
  }

  public String getLimitSwitch() {
    String limitDirection = "no limit hit";
    switch (limitSwitchTriggered) {
      case RIGHT:
        limitDirection = "right";
        break;
      case LEFT:
        limitDirection = "left";
        break;
      case NULL:
        limitDirection = "no limit hit";
        break;
    }
    return limitDirection;
  }

  public void setHoodZero() {
    System.out.println("turret hood encoder zeroed");
    hoodMotor.getEncoder().setPosition(0);
  }
}
