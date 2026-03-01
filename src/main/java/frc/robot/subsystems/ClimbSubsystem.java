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
import frc.robot.util.TuningUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax deploy_motor;
  private final SparkMaxConfig deployConfig;
  // the main mechanism object
  LoggedMechanism2d mech = new LoggedMechanism2d(0.7366, 0.3048);
  // the mechanism root node
  LoggedMechanismRoot2d root = mech.getRoot("Climb", 0.0, 0.1);
  LoggedMechanismLigament2d climbermechanism;
  double currentSetPoint = ClimbConstants.RETRACTED_DEPLOY_POSITION;
  TuningUtil climbKp = new TuningUtil("/Tuning/climb/climbKp", 0.0);
  TuningUtil climbKd = new TuningUtil("/Tuning/climb/climbKd", 0.0);

  public ClimbSubsystem() {

    deploy_motor = new SparkMax(ClimbConstants.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    deployConfig = new SparkMaxConfig();
    deployConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);
    deployConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    deployConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(climbKp.getValue(), 0.0, climbKd.getValue());
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
    climbermechanism = root.append((new LoggedMechanismLigament2d("Climbarm", 0.43, 90)));
  }

  public void deploy() {
    System.out.println("deploying climb mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(ClimbConstants.EXTENDED_DEPLOY_POSITION, ControlType.kPosition);
    currentSetPoint = ClimbConstants.EXTENDED_DEPLOY_POSITION;
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> deploy());
  }

  public void retract() {
    System.out.println("retracting climb mechanism");
    deploy_motor
        .getClosedLoopController()
        .setSetpoint(ClimbConstants.RETRACTED_DEPLOY_POSITION, ControlType.kPosition);
    currentSetPoint = ClimbConstants.RETRACTED_DEPLOY_POSITION;
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> retract());
  }

  public void testClimbVoltage(double volts) {
    deploy_motor.setVoltage(volts);
  }

  public void makeSafe() {
    retract();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbKp
        .get()
        .ifPresent(
            kp -> {
              System.out.println("updated climb kp");
              deployConfig.closedLoop.pid(kp, 0.0, climbKd.getValue());
              deploy_motor.configure(
                  deployConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    climbKd
        .get()
        .ifPresent(
            kd -> {
              System.out.println("updated climb kd");
              deployConfig.closedLoop.pid(climbKp.getValue(), 0.0, kd);
              deploy_motor.configure(
                  deployConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    Logger.recordOutput("Climber/Position", deploy_motor.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    if (currentSetPoint == ClimbConstants.EXTENDED_DEPLOY_POSITION) {
      if (climbermechanism.getLength() < ClimbConstants.extended_length) {
        climbermechanism.setLength(
            climbermechanism.getLength() + (ClimbConstants.extensionSpeed / 50));
      }
    } else {
      if (currentSetPoint == ClimbConstants.RETRACTED_DEPLOY_POSITION) {
        if (climbermechanism.getLength() > ClimbConstants.retracted_length) {
          climbermechanism.setLength(
              climbermechanism.getLength() - (ClimbConstants.extensionSpeed / 50));
        }
      }
    }
    Logger.recordOutput("Climber/Mech", mech);
  }
}
