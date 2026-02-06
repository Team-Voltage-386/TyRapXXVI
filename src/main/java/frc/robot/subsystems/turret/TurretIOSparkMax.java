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

  private final SparkMax yawMotor = new SparkMax(TurretConstants.turretCanId, MotorType.kBrushless);
  private final RelativeEncoder yawEncoder = yawMotor.getEncoder();

  private final SparkMaxConfig yawConfig;

  TuningUtil yawKp = new TuningUtil("/Tuning/turret/yawKp", .0);
  TuningUtil yawKd = new TuningUtil("/Tuning/turret/yawKd", 0.0);

  public TurretIOSparkMax() {
    yawConfig = new SparkMaxConfig();
    yawConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
    yawConfig
        .encoder
        .uvwAverageDepth(2)
        .positionConversionFactor(gearRatioPerRot)
        .velocityConversionFactor(gearRatioPerRot);
    yawConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(7.5, 0.0, 4.0);
    yawConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        5,
        () ->
            yawMotor.configure(
                yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    yawEncoder.setPosition(0);
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
    inputs.connected = true;
    inputs.turretYaw = Rotation2d.fromRotations(yawEncoder.getPosition());
    inputs.turretPitch = Rotation2d.kZero;
    double velocity = yawEncoder.getVelocity();
    double setpoint = yawMotor.getClosedLoopController().getSetpoint();
    double velocitySetpoint = yawMotor.getClosedLoopController().getMAXMotionSetpointVelocity();
    double positionSetpoint = yawMotor.getClosedLoopController().getMAXMotionSetpointPosition();
    Logger.recordOutput("/Shooter/Turret/Setpoint", setpoint);
    Logger.recordOutput("/Shooter/Turret/VelocitySetpoint", velocitySetpoint);
    Logger.recordOutput(
        "/Shooter/Turret/PositionSetpoint", Rotation2d.fromRotations(positionSetpoint));
    Logger.recordOutput("/Shooter/Turret/AppliedOutput", yawMotor.getAppliedOutput());
    Logger.recordOutput("/Shooter/Turret/Velocity", velocity);
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    double desiredAngle =
        MathUtil.clamp(position.getRotations(), turretMinAngleRot, turretMaxAngleRot);

    Logger.recordOutput("/Shooter/Turret/DesiredAngle", Rotation2d.fromRotations(desiredAngle));

    yawMotor
        .getClosedLoopController()
        .setSetpoint(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void testTurretVoltage(double volts) {
    yawMotor.setVoltage(volts);
  }

  /* Set the turret pitch to the specified position. */
  @Override
  public void setTurretPitch(Rotation2d position) {
    // No pitch control implemented
  }

  public void setZero() {
    System.out.println("turret encoder zeroed");
    yawEncoder.setPosition(0);
  }
}
