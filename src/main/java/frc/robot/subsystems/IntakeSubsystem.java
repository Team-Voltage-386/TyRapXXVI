package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax retrieval_motor;

  private final SparkMax deploy_motor;

  public IntakeSubsystem() {
    retrieval_motor = new SparkMax(IntakeConstants.RETRIEVAL_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig retrievalConfig = new SparkMaxConfig();
    retrievalConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(10).voltageCompensation(12.0);
    retrievalConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    retrievalConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    retrievalConfig
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
            retrieval_motor.configure(
                retrievalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    deploy_motor = new SparkMax(IntakeConstants.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig deployConfig = new SparkMaxConfig();
    deployConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(10).voltageCompensation(12.0);
    deployConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    deployConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeConstants.deployKp, 0.0, IntakeConstants.deployKd);
    deployConfig
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
            deploy_motor.configure(
                deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    deploy_motor.getEncoder().setPosition(IntakeConstants.RETRACTED_DEPLOY_POSITION);
  }

  public void deploy() {
    System.out.println("deploying intake mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(IntakeConstants.EXTENDED_DEPLOY_POSITION, ControlType.kPosition);
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> deploy());
  }

  public void retract() {
    System.out.println("retracting intake mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(IntakeConstants.RETRACTED_DEPLOY_POSITION, ControlType.kPosition);
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> retract());
  }

  public void takeIn() {
    System.out.println("taking in balls");
    retrieval_motor.setVoltage(IntakeConstants.RETRIEVAL_MOTOR_VOLTAGE);
  }

  public Command takeInCommand() {
    return Commands.runOnce(() -> takeIn());
  }

  public void stopMotor() {
    System.out.println("Stopping motor");
    retrieval_motor.set(0);
  }

  public Command stopMotorCommand() {
    return Commands.runOnce(() -> stopMotor());
  }

  public void reverse() {
    System.out.println("Reversing motor");
    retrieval_motor.setVoltage(IntakeConstants.RETRIEVAL_MOTOR_VOLTAGE * -1);
  }

  public Command reverseCommand() {
    return Commands.startEnd(() -> reverse(), () -> stopMotor());
  }

  public boolean isMotorStalled() {
    return retrieval_motor.getOutputCurrent() > -1;
  }

  public void makeSafe() {
    stopMotor();
    retract();
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
