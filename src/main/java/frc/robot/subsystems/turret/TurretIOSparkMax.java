package frc.robot.subsystems.turret;

import static frc.robot.constants.jr.TurretConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class TurretIOSparkMax implements TurretIO {

  private final SparkMax yawMotor =
      new SparkMax(TurretConstants.turretYawCanId, MotorType.kBrushless);
  private final SparkMax hoodMotor =
      new SparkMax(TurretConstants.turretPitchCanId, MotorType.kBrushless);

  private final RelativeEncoder yawEncoder = yawMotor.getAlternateEncoder();
  private final SparkMaxConfig yawConfig;
  private final SparkMaxConfig hoodConfig;

  private static final int kCPR = 8192;

  // Turret Limit Input
  DigitalInput turretLimitInput = new DigitalInput(9);

  TuningUtil yawKp = new TuningUtil("/Tuning/turret/yawKp", 0.0);
  TuningUtil yawKd = new TuningUtil("/Tuning/turret/yawKd", 0.0);
  TuningUtil hoodKp = new TuningUtil("/Tuning/turret/hoodKp", 0.9);
  TuningUtil hoodKd = new TuningUtil("/Tuning/turret/hoodKd", 0.0);
  TuningUtil turretRightLimit = new TuningUtil("/Tuning/turret/rightLimit", 0);
  TuningUtil turretLeftLimit = new TuningUtil("/Tuning/turret/leftLimit", 0);
  protected boolean clockwiseLimitHit = false;
  protected boolean counterclockwiseLimitHit = false;

  public TurretIOSparkMax() {
    yawConfig = new SparkMaxConfig();
    yawConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
    /*yawConfig
    .encoder
    .uvwAverageDepth(2)
    .positionConversionFactor(gearRatioPerRot)
    .velocityConversionFactor(gearRatioPerRot); */
    yawConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(7.5, 0.0, 4.0);
    yawConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    yawConfig.inverted(true);
    yawConfig.alternateEncoder.countsPerRevolution(kCPR);
    yawConfig
        .alternateEncoder
        .velocityConversionFactor(gearRatioPerRot)
        .positionConversionFactor(gearRatioPerRot);

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
              yawConfig.closedLoop.pid(kp, 0.0, yawKd.getValue());
              yawMotor.configure(
                  yawConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    yawKd
        .get()
        .ifPresent(
            kd -> {
              System.out.println("updated turret kd");
              yawConfig.closedLoop.pid(yawKp.getValue(), 0.0, kd);
              yawMotor.configure(
                  yawConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

    Logger.recordOutput(
        "/Shooter/Turret/InternalEncoderYaw",
        Rotation2d.fromRotations(yawMotor.getEncoder().getPosition()));

    double velocity = yawEncoder.getVelocity();
    double setpoint = yawMotor.getClosedLoopController().getSetpoint();
    double hoodsetpoint = hoodMotor.getClosedLoopController().getSetpoint();

    Logger.recordOutput("/Shooter/Turret/Setpoint", setpoint);
    Logger.recordOutput("/Shooter/Turret/AppliedOutput", yawMotor.getAppliedOutput());
    Logger.recordOutput("/Shooter/Turret/Velocity", velocity);
    Logger.recordOutput("/Shooter/Turret/Current", yawMotor.getOutputCurrent());
    Logger.recordOutput("/Shooter/Turret/LimitSwitchTrue", inputs.turretLimitTrue);
    Logger.recordOutput("/Shooter/Hood/AppliedOutput", hoodMotor.getAppliedOutput());
    Logger.recordOutput("/Shooter/Hood/Setpoint", hoodsetpoint);
    Logger.recordOutput("/Shooter/Hood/Current", hoodMotor.getOutputCurrent());
    Logger.recordOutput("/Shooter/Hood/position", hoodMotor.getEncoder().getPosition());
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    double desiredPositionDeg = position.getDegrees();
    Logger.recordOutput("/Shooter/Turret/DesiredAngleRobotCtr", position);
    Rotation2d offsetPos =
        Rotation2d.fromDegrees(
            minus180To180(desiredPositionDeg + TurretConstants.turretCenterOffsetDeg));
    double desiredAngle =
        MathUtil.clamp(offsetPos.getRotations(), turretMinAngleRot, turretMaxAngleRot);

    Logger.recordOutput(
        "/Shooter/Turret/DesiredAngleTurretCtr", Rotation2d.fromRotations(desiredAngle));

    yawMotor
        .getClosedLoopController()
        .setSetpoint(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double zeroTo360(double angle) {
    double result = angle % 360;
    if (result < 0) {
      result += 360;
    }
    return result;
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
    yawMotor.setVoltage(volts);
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
        -1.0 * Rotation2d.fromDegrees(TurretConstants.turretCenterOffsetDeg).getRotations());
  }

  public void setHoodZero() {
    System.out.println("turret hood encoder zeroed");
    hoodMotor.getEncoder().setPosition(0);
  }
}
