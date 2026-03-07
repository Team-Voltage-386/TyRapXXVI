package frc.robot.constants.jr;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {
  public static final int RETRIEVAL_MOTOR_CAN_ID = 2;
  public static final int DEPLOY_MOTOR_CAN_ID = 7;
  public static final double RETRIEVAL_MOTOR_VOLTAGE = 3.0;
  public static final double EXTENDED_DEPLOY_POSITION = 2.0;
  public static final double RETRACTED_DEPLOY_POSITION = 0.0;
  public static final double deployKp = 1.0;
  public static final double deployKd = 0.0;
  public static final Rotation2d extendedAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d retractedAngle = Rotation2d.fromDegrees(90);
  public static final double speed = 180;
}
