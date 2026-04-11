package frc.robot.constants.jr;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {

  public static final int RETRIEVAL_MOTOR_CAN_ID = 32;
  public static final int DEPLOY_MOTOR_CAN_ID = 27;
  public static final double RETRIEVAL_MOTOR_VOLTAGE = -8.5;
  public static final double DEPLOY_MANUAL_VOLTAGE = 3.0;
  public static final double EXTENDED_DEPLOY_POSITION = -19;
  public static final double HALF_DEPLOY_POSITION = -8;
  public static final double RETRACTED_DEPLOY_POSITION = 0.0;
  public static final double deployKp = 0.3;
  public static final double deployKd = 0.0;
  public static final Rotation2d extendedAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d retractedAngle = Rotation2d.fromDegrees(90);
  public static final double speed = 180;
  public static final double deployTimeSec = 0.5;
  // public static final double deployRate = -1 * EXTENDED_DEPLOY_POSITION / deployTimeSec;
  public static final double deployRate = -260.0;
  public static final double deployIncPerStepFast = deployRate * 0.02;
  public static final double deployIncPerStepSlow = deployIncPerStepFast * 0.3;
  public static final double deploySlowdownPointUp = 6.5;
  public static final double deploySlowdownPointDown = 10.7;
}
