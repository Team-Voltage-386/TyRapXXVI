package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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

public class SpindexerSubsystem extends SubsystemBase {

  private final SparkFlex spindexer_motor;
  private final SparkFlex feeder_motor;
  private final SparkMax antijam_motor;
  public boolean feederOn = false;
  public boolean reverse = false;

  TuningUtil spindexerVolts =
      new TuningUtil("Spindexer/spindexerVolts", SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE);
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
  }

  public Command spindexerOffCommand() {
    return Commands.runOnce(() -> spindexerOff());
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
    spindexer_motor.setVoltage(-SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE);
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

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
