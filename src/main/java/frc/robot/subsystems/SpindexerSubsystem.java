package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {

  private final SparkFlex spindexer_motor;
  private final SparkFlex agitator_motor;
  private final SparkFlex feeder_motor;
  public boolean feederOn = false;
  public boolean reverse = true;

  public SpindexerSubsystem() {
    spindexer_motor =
        new SparkFlex(SpindexerConstants.SPINDEXER_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);
    spindexerConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    spindexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
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
    SparkFlexConfig agitatorConfig = new SparkFlexConfig();
    agitatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);
    agitatorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    agitatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
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
  }

  public void spindexerOn() {
    System.out.println("turning on spindexer");
    reverse = false;
    spindexer_motor.setVoltage(-SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE);
    agitatorOn();
  }

  public Command spindexerOnCommand() {
    return Commands.runOnce(() -> spindexerOn());
  }

  public void spindexerOff() {
    // System.out.println("turning off spindexer");
    reverse = false;
    spindexer_motor.set(0);
    agitatorOff();
  }

  public Command spindexerOffCommand() {
    return Commands.runOnce(() -> spindexerOff());
  }

  public void agitatorOn() {
    System.out.println("turning on agitator");
    reverse = false;
    agitator_motor.setVoltage(-SpindexerConstants.AGITATOR_MOTOR_VOLTAGE);
  }

  public Command agitatorOnCommand() {
    return Commands.runOnce(() -> agitatorOn());
  }

  public void agitatorOff() {
    // System.out.println("turning off spindexer");
    reverse = false;
    agitator_motor.set(0);
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
