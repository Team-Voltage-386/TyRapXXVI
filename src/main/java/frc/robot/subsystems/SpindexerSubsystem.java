package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {

  // private final SparkMax spindexer_motor;

  private final SparkMax feeder_motor;
  public boolean feederOn = false;

  public SpindexerSubsystem() {
    // spindexer_motor = new SparkMax(SpindexerConstants.SPINDEXER_MOTOR_CAN_ID,
    // MotorType.kBrushless);
    SparkMaxConfig spindexerConfig = new SparkMaxConfig();
    spindexerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(10).voltageCompensation(12.0);
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
    /*tryUntilOk(
    5,
    () ->
        spindexer_motor.configure(
            spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)); */

    feeder_motor = new SparkMax(SpindexerConstants.FEEDER_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(10).voltageCompensation(12.0);
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
    feederOn = true;
    // spindexer_motor.setVoltage(SpindexerConstants.SPINDEXER_MOTOR_VOLTAGE);
  }

  public Command spindexerOnCommand() {
    return Commands.runOnce(() -> spindexerOn());
  }

  public void spindexerOff() {
    System.out.println("turning off spindexer");
    feederOn = false;
    // spindexer_motor.set(0);
  }

  public Command spindexerOffCommand() {
    return Commands.runOnce(() -> spindexerOff());
  }

  public void feederOn() {
    System.out.println("turning on feeder");
    feeder_motor.setVoltage(SpindexerConstants.FEEDER_MOTOR_VOLTAGE);
  }

  public Command feederOnCommand() {
    return Commands.runOnce(() -> feederOn());
  }

  public void feederOff() {
    System.out.println("turning off feeder");
    feeder_motor.set(0);
  }

  public Command feederOffCommand() {
    return Commands.runOnce(() -> feederOff());
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
