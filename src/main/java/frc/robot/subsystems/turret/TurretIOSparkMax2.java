package frc.robot.subsystems.turret;

import static frc.robot.constants.jr.TurretConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
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
  TuningUtil hoodKp = new TuningUtil("/Tuning/turret/hoodKp", 0.9);
  TuningUtil hoodKd = new TuningUtil("/Tuning/turret/hoodKd", 0.0);
  TuningUtil turretRightLimit = new TuningUtil("/Tuning/turret/rightLimit", 0);
  TuningUtil turretLeftLimit = new TuningUtil("/Tuning/turret/leftLimit", 0);
  protected boolean clockwiseLimitHit = false;
  protected boolean counterclockwiseLimitHit = false;
  protected double desiredAngle = 0.0;
  protected boolean manualMode = true;

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
      new SimpleMotorFeedforward(TurretConstants.turretKs, TurretConstants.turretKv);

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

    tryUntilOk(
        5,
        () ->
            yawMotor.configure(
                yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    setYawZero();

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
    inputs.turretYaw = yawTurretCenter.minus(TurretConstants.turretCenterOffsetRot);
    inputs.turretPitch = Rotation2d.fromRotations(hoodMotor.getEncoder().getPosition());
    inputs.turretLimitTrue = !turretLimitInput.get();

    double velocity = yawEncoder.getVelocity();
    double setpoint = yawMotor.getClosedLoopController().getSetpoint();
    double hoodsetpoint = hoodMotor.getClosedLoopController().getSetpoint();

    if (!manualMode) {
      if (Math.abs(desiredAngle - yawEncoder.getPosition()) < (1.0 / 360.0)) {
        yawMotor.setVoltage(0);
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
        // if (outVoltage > 0) {
        //   outVoltage += yawFeedforward.getKs();
        // } else if (outVoltage < 0) {
        //   outVoltage -= yawFeedforward.getKs();
        // }
        Logger.recordOutput("Shooter/Turret/pidVal", pidVal);
        Logger.recordOutput("Shooter/Turret/feedForwardVal", feedforwardVal);
        Logger.recordOutput("Shooter/Turret/outVoltage", outVoltage);
        Logger.recordOutput(
            "Shooter/Turret/profiledSetpoint", yawController.getSetpoint().position);
        Logger.recordOutput(
            "Shooter/Turret/profiledVelSetpoint", yawController.getSetpoint().velocity);
        yawMotor.setVoltage(outVoltage);
        lastYawSpeed = yawController.getSetpoint().velocity / 60.0;
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
    manualMode = false;
    Logger.recordOutput("/Shooter/Turret/DesiredAngleRobotCtr", position);
    double wrappedPositionDeg = position.getDegrees();
    if (position.getRotations() > TurretConstants.turretMaxAngleRot) {
      System.out.println("Wrapping angle around negative");
      wrappedPositionDeg = wrappedPositionDeg - 360.0;
    } else if (position.getRotations() < TurretConstants.turretMinAngleRot) {
      System.out.println("Wrapping angle around positive");
      wrappedPositionDeg = wrappedPositionDeg + 360.0;
    }
    Logger.recordOutput("/Shooter/Turret/DesiredAngleWrapped", wrappedPositionDeg);
    Logger.recordOutput(
        "/Shooter/Turret/DesiredAngleWrappedRot",
        Rotation2d.fromDegrees(wrappedPositionDeg).getRotations());
    desiredAngle =
        MathUtil.clamp(
            Rotation2d.fromDegrees(wrappedPositionDeg).getRotations(),
            TurretConstants.turretMinAngleRot,
            TurretConstants.turretMaxAngleRot);

    Logger.recordOutput(
        "/Shooter/Turret/DesiredAngleTurretCtr", Rotation2d.fromRotations(desiredAngle));
    /*yawMotor
    .getClosedLoopController()
    .setSetpoint(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);*/
  }

  public double zeroTo360(double angle) {
    return (angle + 360.0) / 360.0;
  }

  public double minus180To180(double angleDeg) {
    double angle360 = zeroTo360(angleDeg);
    if (angle360 > 180) {
      return angle360 - 360;
    } else {
      return angle360;
    }
  }

  public void testTurretVoltage(double volts) {
    manualMode = true;
    if (debouncer.calculate(!turretLimitInput.get())) {
      // check velocity directions
      if (yawEncoder.getVelocity() > 0 && limitSwitchTriggered == WhichLimit.NULL) {
        limitSwitchTriggered = WhichLimit.LEFT;
      }
      if (yawEncoder.getVelocity() < 0 && limitSwitchTriggered == WhichLimit.NULL) {
        limitSwitchTriggered = WhichLimit.RIGHT;
      }
    } else {
      limitSwitchTriggered = WhichLimit.NULL;
      yawMotor.setVoltage(volts);
    }
    if (limitSwitchTriggered == WhichLimit.RIGHT) {
      System.out.println(
          "Right limit switch triggered, cannot do volts in this direction any further");
      yawMotor.setVoltage(MathUtil.clamp(volts, 0, TurretConstants.maxYawVoltage));
    }
    if (limitSwitchTriggered == WhichLimit.LEFT) {
      System.out.println(
          "Left limit switch triggered, cannot do volts in this direction any further");
      yawMotor.setVoltage(MathUtil.clamp(volts, -TurretConstants.maxYawVoltage, 0));
    }
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

  public void setYawZero() {
    System.out.println("turret encoder zeroed");
    yawEncoder.setPosition(
        Rotation2d.fromDegrees(TurretConstants.turretCenterOffsetDeg).getRotations());
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
