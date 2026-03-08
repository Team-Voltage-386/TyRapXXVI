package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.jr.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkFlex retrieval_motor;

  private final SparkMax deploy_motor;

  public IntakeIOSparkMax() {
    retrieval_motor = new SparkFlex(IntakeConstants.RETRIEVAL_MOTOR_CAN_ID, MotorType.kBrushless);
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
    deployConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);
    deployConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    deployConfig.softLimit.forwardSoftLimitEnabled(false);
    deployConfig.softLimit.reverseSoftLimitEnabled(false);
    deployConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    deployConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
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

  public void retract() {
    System.out.println("retracting intake mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(IntakeConstants.RETRACTED_DEPLOY_POSITION, ControlType.kPosition);
  }

  public void takeIn() {
    System.out.println("taking in balls");
    retrieval_motor.setVoltage(IntakeConstants.RETRIEVAL_MOTOR_VOLTAGE);
  }

  public void stopMotor() {
    System.out.println("Stopping motor");
    retrieval_motor.set(0);
  }

  public void reverse() {
    System.out.println("Reversing motor");
    retrieval_motor.setVoltage(IntakeConstants.RETRIEVAL_MOTOR_VOLTAGE * -1);
  }

  public boolean isMotorStalled() {
    return retrieval_motor.getOutputCurrent() > -1;
  }

  public void makeSafe() {
    stopMotor();
    retract();
  }

  public void testDeployVoltage(double voltage) {
    /*if (voltage > 0) {
      if (deploy_motor.getEncoder().getPosition() > -IntakeConstants.EXTENDED_DEPLOY_POSITION) {
        voltage = 0;
      }
    } else if (voltage < 0) {
      if (deploy_motor.getEncoder().getPosition() <= IntakeConstants.RETRACTED_DEPLOY_POSITION) {
        voltage = 0;
      }
    } else {
      deploy_motor.setVoltage(voltage);
    }*/
    deploy_motor.setVoltage(voltage);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = true;
    inputs.deployed =
        Math.abs(deploy_motor.getEncoder().getPosition() - IntakeConstants.EXTENDED_DEPLOY_POSITION)
            < 5;
    inputs.intakingState =
        retrieval_motor.getAppliedOutput() > 0.1
            ? IntakingState.INTAKING
            : retrieval_motor.getAppliedOutput() < -0.1
                ? IntakingState.REVERSE
                : IntakingState.STOPPED;
    Logger.recordOutput("Intake/DeployMotor/Position", deploy_motor.getEncoder().getPosition());
    Logger.recordOutput(
        "Intake/DeployMotor/AbsolutePosition", deploy_motor.getAbsoluteEncoder().getPosition());
  }
}
