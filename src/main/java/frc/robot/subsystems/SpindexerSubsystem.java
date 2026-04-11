package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.SpindexerConstants;
import frc.robot.util.TuningUtil;
import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {

  private final SparkFlex spindexer_motor;
  private final SparkFlex agitator_motor;
  private final SparkFlex feeder_motor;
  private final SparkMax antijam_motor;
  public boolean feederOn = false;
  public boolean reverse = false;

  SparkFlexConfig agitatorConfig;

  TuningUtil agitatorKp =
      new TuningUtil("/Tuning/spindexer/agitatorKp", SpindexerConstants.AGITATOR_KP);
  TuningUtil agitatorKd =
      new TuningUtil("/Tuning/spindexer/agitatorKd", SpindexerConstants.AGITATOR_KD);
  TuningUtil agitatorRpm =
      new TuningUtil("/Tuning/spindexer/agitatorRpm", SpindexerConstants.AGITATOR_RPM);
  double agitatorRpmVal = agitatorRpm.getValue();
  double agitatorSetpoint = 0.0;
  double agitatorIncrement = (agitatorRpmVal / 60.0) / 50.0;

  TuningUtil spindexerVolts =
      new TuningUtil("Spindexer/spindexerVolts", SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE);
  TuningUtil agitatorVolts =
      new TuningUtil("Spindexer/agitatorVolts", SpindexerConstants.AGITATOR_MOTOR_VOLTAGE);
  TuningUtil antijamVolts =
      new TuningUtil("Spindexer/antijamVolts", SpindexerConstants.ANTIJAM_VOLTAGE);

  public SpindexerSubsystem() {
    spindexer_motor =
        new SparkFlex(SpindexerConstants.SPINDEXER_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);
    spindexerConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    spindexerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            spindexer_motor.configure(
                spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    agitator_motor = new SparkFlex(SpindexerConstants.AGITATOR_MOTOR_CAN_ID, MotorType.kBrushless);
    agitatorConfig = new SparkFlexConfig();
    agitatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30).voltageCompensation(12.0);
    agitatorConfig
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2)
        .positionConversionFactor(SpindexerConstants.agitatorGearRatio)
        .velocityConversionFactor(SpindexerConstants.agitatorGearRatio);
    agitatorConfig.closedLoop.pid(
        agitatorKp.getValue(), 0.0, agitatorKd.getValue(), ClosedLoopSlot.kSlot0);
    agitatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0.0, 1.0);
    agitatorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            agitator_motor.configure(
                agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    feeder_motor = new SparkFlex(SpindexerConstants.FEEDER_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80).voltageCompensation(12.0);
    feederConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    feederConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            feeder_motor.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    antijam_motor = new SparkMax(SpindexerConstants.ANTIJAM_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig antijamConfig = new SparkMaxConfig();
    antijamConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30).voltageCompensation(12.0);
    antijamConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    antijamConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        5,
        () ->
            antijam_motor.configure(
                antijamConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void spindexerOn() {
    System.out.println("turning on spindexer");
    reverse = false;
    spindexer_motor.setVoltage(spindexerVolts.getValue());
    antijam_motor.setVoltage(antijamVolts.getValue());
    agitatorOn();
  }

  public boolean isSpindexerOn() {
    return spindexer_motor.getAppliedOutput() != 0;
  }

  public Command spindexerOnCommand() {
    return Commands.runOnce(() -> spindexerOn());
  }

  public void spindexerOff() {
    // System.out.println("turning off spindexer");
    reverse = false;
    spindexer_motor.set(0);
    antijam_motor.set(0);
    agitatorOff();
  }

  public Command spindexerOffCommand() {
    return Commands.runOnce(() -> spindexerOff());
  }

  public void agitatorOn() {
    System.out.println("turning on agitator");
    reverse = false;
    agitator_motor.getEncoder().setPosition(0);
    agitatorSetpoint = 0.0;
    // agitator_motor.setVoltage(agitatorVolts.getValue());
  }

  public Command agitatorOnCommand() {
    return Commands.runOnce(() -> agitatorOn());
  }

  public void agitatorOff() {
    // System.out.println("turning off spindexer");
    reverse = false;
    // agitator_motor.set(0);
  }

  public Command agitatorOffCommand() {
    return Commands.runOnce(() -> agitatorOff());
  }

  public void feederOn() {
    System.out.println("turning on feeder");
    feederOn = true;
    reverse = false;
    feeder_motor.setVoltage(-SpindexerConstants.FEEDER_MOTOR_VOLTAGE);
  }

  public void feederReverse() {
    System.out.println("reversing feeder");
    reverse = true;
    spindexer_motor.setVoltage(SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE);
    feeder_motor.setVoltage(SpindexerConstants.FEEDER_MOTOR_VOLTAGE);
    antijam_motor.setVoltage(-antijamVolts.getValue());
  }

  public Command feederOnCommand() {
    return Commands.runOnce(() -> feederOn());
  }

  public Command feederReverseCommand() {
    return Commands.runOnce(() -> feederReverse());
  }

  public void feederOff() {
    System.out.println("turning off feeder");
    feederOn = false;
    feeder_motor.set(0);
  }

  public Command feederOffCommand() {
    return Commands.runOnce(() -> feederOff());
  }

  public boolean isFeederOn() {
    return feederOn;
  }

  public boolean turnFeederFalse() {
    return feederOn = false;
  }

  public Command feederFalseCommand() {
    return Commands.runOnce(() -> turnFeederFalse());
  }

  public boolean isReverse() {
    return reverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    agitatorKp
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated agitator kp");
              agitatorConfig.closedLoop.p(kp, ClosedLoopSlot.kSlot0);
              agitator_motor.configure(
                  agitatorConfig,
                  ResetMode.kNoResetSafeParameters,
                  PersistMode.kNoPersistParameters);
            });

    agitatorKd
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated agitator kd");
              agitatorConfig.closedLoop.d(kp, ClosedLoopSlot.kSlot0);
              agitator_motor.configure(
                  agitatorConfig,
                  ResetMode.kNoResetSafeParameters,
                  PersistMode.kNoPersistParameters);
            });

    agitatorRpm
        .get()
        .ifPresent(
            rpm -> {
              System.out.println("updated agitator rpm");
              agitatorRpmVal = rpm;
              agitatorIncrement = (agitatorRpmVal / 60.0) / 50.0;
            });

    if (spindexer_motor.getAppliedOutput() != 0) {
      if (reverse) {
        agitatorSetpoint -= agitatorIncrement;
      } else {
        agitatorSetpoint += agitatorIncrement;
      }
      if (agitatorSetpoint > 1.0) {
        agitatorSetpoint -= 1.0;
      } else if (agitatorSetpoint < 0.0) {
        agitatorSetpoint += 1.0;
      }
      agitator_motor.getClosedLoopController().setSetpoint(agitatorSetpoint, ControlType.kPosition);
    } else {
      agitator_motor.setVoltage(0.0);
    }

    Logger.recordOutput("Spindexer/Agitator/Velocity", agitator_motor.getEncoder().getVelocity());
    Logger.recordOutput("Spindexer/Agitator/Position", agitator_motor.getEncoder().getPosition());
    Logger.recordOutput("Spindexer/Agitator/Setpoint", agitatorSetpoint);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
