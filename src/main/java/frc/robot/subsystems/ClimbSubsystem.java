package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.jr.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax deploy_motor;

  public ClimbSubsystem() {

    deploy_motor = new SparkMax(ClimbConstants.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig deployConfig = new SparkMaxConfig();
    deployConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(10).voltageCompensation(12.0);
    deployConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    deployConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ClimbConstants.deployKp, 0.0, ClimbConstants.deployKd);
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
    deploy_motor.getEncoder().setPosition(ClimbConstants.RETRACTED_DEPLOY_POSITION);
  }

  public void deploy() {
    System.out.println("deploying climb mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(ClimbConstants.EXTENDED_DEPLOY_POSITION, ControlType.kPosition);
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> deploy());
  }

  public void retract() {
    System.out.println("retracting climb mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(ClimbConstants.RETRACTED_DEPLOY_POSITION, ControlType.kPosition);
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> retract());
  }

  public void makeSafe() {
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
