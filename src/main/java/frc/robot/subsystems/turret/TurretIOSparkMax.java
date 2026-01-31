package frc.robot.subsystems.turret;

import static frc.robot.constants.jr.TurretConstants.zeroRotRadians;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  // NEEDS the absolute encoder board plugged into MAX controller--not normal board.
  private final AbsoluteEncoder yawEncoder = yawMotor.getAbsoluteEncoder();

  private SparkMaxConfig yawConfig;

  TuningUtil yawKp = new TuningUtil("/Tuning/turret/yawKp", .7);
  TuningUtil yawKd = new TuningUtil("/Tuning/turret/yawKd", 0.0);

  public TurretIOSparkMax() {
    yawConfig = new SparkMaxConfig();
    yawConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);
    yawConfig
        .absoluteEncoder
        .inverted(false)
        .averageDepth(2)
        .zeroOffset(Units.radiansToRotations(zeroRotRadians));
    yawConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(.7, 0.0, 0.0);
    yawConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        5,
        () ->
            yawMotor.configure(
                yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(TurretIOInputs inputs) {
    yawKp
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated turret kp");
              yawConfig.closedLoop.pid(kp, 0.0, yawKd.getValue());
              yawMotor.configure(
                  yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            });
    yawKd
        .get()
        .ifPresent(
            kd -> {
              System.out.println("updated turret kd");
              yawConfig.closedLoop.pid(yawKp.getValue(), 0.0, kd);
              yawMotor.configure(
                  yawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            });
    inputs.connected = true;
    inputs.turretYaw = Rotation2d.fromRotations(yawEncoder.getPosition());
    inputs.turretPitch = Rotation2d.kZero;
  }

  /** Set the turret yaw to the specified position. */
  @Override
  public void setTurretYaw(Rotation2d position) {
    // TODO: FF

    Logger.recordOutput("/Shooter/Turret/DesiredAngle", position);
    yawMotor
        .getClosedLoopController()
        .setSetpoint(position.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /* Set the turret pitch to the specified position. */
  @Override
  public void setTurretPitch(Rotation2d position) {
    // No pitch control implemented
  }
}
