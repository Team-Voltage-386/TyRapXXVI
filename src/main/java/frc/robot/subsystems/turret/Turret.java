package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.jr.TurretConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Handles turret control and aiming functionality */
public class Turret extends SubsystemBase {

  public final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Pose3d[] turretVisual = new Pose3d[2];

  private final Supplier<Pose2d> dtPose;
  private Pose3d currentTargetPose = new Pose3d(Constants.blueHubPose, Rotation3d.kZero);
  private final Flywheel flywheel;
  private ShotCalculation shotCalculation;

  private boolean autoAimEnabled = false;
  private boolean manualMode = false;
  protected boolean isScoring = true;

  private Rotation2d manualPitch = new Rotation2d(TurretConstants.turretMaxHoodAngle);
  private Rotation2d calculatedPitch;
  private Supplier<Boolean> isShootingSupplier;
  private Supplier<Boolean> isInAlliance;
  private Supplier<Boolean> verticalHalfofField;
  private Supplier<Double> triggerSupplier;

  public Turret(
      TurretIO io,
      Supplier<Pose2d> dtPose,
      Flywheel flywheel,
      ShotCalculation shotCalculation,
      Supplier<Boolean> isShootingSupplier,
      Supplier<Boolean> isInAlliance,
      Supplier<Boolean> verticalHalfofField,
      Supplier<Double> triggerSupplier) {
    this.io = io;
    this.dtPose = dtPose;
    this.flywheel = flywheel;
    this.shotCalculation = shotCalculation;
    this.calculatedPitch = Rotation2d.fromDegrees(TurretConstants.turretMaxHoodAngle);
    this.isShootingSupplier = isShootingSupplier;
    this.isInAlliance = isInAlliance;
    this.verticalHalfofField = verticalHalfofField;
    this.triggerSupplier = triggerSupplier;

    io.setTurretPitch(Rotation2d.fromDegrees(TurretConstants.turretMaxHoodAngle));
  }

  public Command manualIncrimentPitch(Rotation2d deltaPitch) {
    return runOnce(
        () -> {
          manualPitch = manualPitch.plus(deltaPitch);
          if (manualPitch.getDegrees() > TurretConstants.turretMaxHoodAngle) {
            manualPitch = Rotation2d.fromDegrees(TurretConstants.turretMaxHoodAngle);
          } else if (manualPitch.getDegrees() < TurretConstants.turretMinHoodAngle) {
            manualPitch = Rotation2d.fromDegrees(TurretConstants.turretMinHoodAngle);
          }
          io.setTurretPitch(manualPitch);
        });
  }

  public Command addPitchCommand(Rotation2d deltaPitch) {
    return runOnce(() -> io.setTurretPitch(inputs.turretPitch.plus(deltaPitch)));
  }

  public Command addYawCommand(Rotation2d deltaYaw) {
    return runOnce(() -> io.setTurretYaw(inputs.turretYaw.plus(deltaYaw)));
  }

  public Command enableAutoAimCommand(Supplier<Pose3d> targetPose) {
    return runOnce(
        () -> {
          autoAimEnabled = true;
          currentTargetPose = targetPose.get();
        });
  }

  public Command disableAutoAimCommand() {
    return runOnce(() -> autoAimEnabled = false);
  }

  public boolean isAutoAimEnabled() {
    return autoAimEnabled;
  }

  public void toggleAutoAim() {
    autoAimEnabled = !autoAimEnabled;
    System.out.println("autoaim: " + autoAimEnabled);
    if (!autoAimEnabled) {
      flywheel.setFlywheelSpeed(0);
    }
  }

  // Stops the Turret Yaw
  public void stopTurretYaw() {
    io.testTurretVoltage(0);
  }

  public Command toggleAutoAimCommand() {
    return runOnce(() -> toggleAutoAim());
  }

  public void aimAtTarget(Pose3d targetPose, boolean isScoring) {
    shotCalculation.setTarget(targetPose.getTranslation().toTranslation2d(), isScoring);
    Pose2d turretFieldPos = shotCalculation.getParameters().lookaheadPose();
    Translation3d turretFieldTrans =
        new Translation3d(
            turretFieldPos.getTranslation().getX(), turretFieldPos.getTranslation().getY(), 0.336);
    Translation3d deltaPos = targetPose.getTranslation().minus(turretFieldTrans);

    double yaw = Math.atan2(deltaPos.getY(), deltaPos.getX());
    calculatedPitch = new Rotation2d(shotCalculation.getParameters().hoodAngle());
    if (isShootingSupplier.get()) {
      io.setTurretPitch(calculatedPitch);
    } else {
      io.setTurretPitch(TurretConstants.turretMaxHoodRot);
    }

    double desiredTurretYaw =
        Rotation2d.fromRadians(yaw).getDegrees() - turretFieldPos.getRotation().getDegrees();
    io.setTurretYaw(limitTurretYaw(desiredTurretYaw));
    Logger.recordOutput("Shooter/Hood/CalculatedPitch", calculatedPitch);
    double shooterWheelRPM = shotCalculation.getParameters().flywheelSpeed();
    flywheel.setFlywheelSpeed(shooterWheelRPM);
    Logger.recordOutput("Shooter/Turret/ShooterWheelRPM", shooterWheelRPM);
    Logger.recordOutput("Shooter/Turret/currentTargetPose", targetPose);
    shotCalculation.clearLaunchingParameters();
  }

  /**
   * Aims the turret to shoot a Fuel through the specified target pose.
   *
   * @param targetPose - The pose to hit with the Fuel.
   * @return The command to aim the turret.
   */
  // deleted aimAtCommand

  public Command adjustPitch(Supplier<Double> pitchDeg) {
    return runOnce(() -> io.setTurretPitch(new Rotation2d(Math.toRadians(pitchDeg.get()))));
  }

  public Command adjustYaw(Supplier<Double> yawDeg) {
    return runOnce(() -> io.setTurretYaw(Rotation2d.fromDegrees(yawDeg.get())));
  }

  public void setTarget() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    if (this.isInAlliance.get()) {
      isScoring = true;
      if (alliance == DriverStation.Alliance.Blue) {
        currentTargetPose = Constants.blueHubPose3d;
      } else {
        currentTargetPose = Constants.redHubPose3d;
      }
    } else {
      isScoring = false;
      if (alliance == DriverStation.Alliance.Blue) {
        if (this.verticalHalfofField.get()) {
          currentTargetPose = Constants.blueLeftCornerPose3d;
        } else {
          currentTargetPose = Constants.blueRightCornerPose3d;
        }
      } else {
        if (this.verticalHalfofField.get()) {
          currentTargetPose = Constants.redRightCornerPose3d;
        } else {
          currentTargetPose = Constants.redLeftCornerPose3d;
        }
      }
    }
  }

  public double zeroTo360(double angle) {
    return (angle + 360.0) % 360;
  }

  public double getAngleDifference(double targetAngle, double currentAngle) {
    double diff = zeroTo360(targetAngle) - zeroTo360(currentAngle);
    if (diff > 180) {
      diff -= 360;
    } else if (diff < -180) {
      diff += 360;
    }
    return diff;
  }

  protected Rotation2d limitTurretYaw(double desiredYawDeg) {
    double desiredYaw360 = zeroTo360(desiredYawDeg);
    // Check if the desired yaw is within the dead zone
    if (Math.abs(getAngleDifference(desiredYaw360, TurretConstants.turretDeadZoneCenterDeg))
        < (TurretConstants.turretDeadZoneHalfWidthDeg)) {
      // If it is, snap to the nearest edge of the dead zone
      if (Math.abs(getAngleDifference(desiredYaw360, TurretConstants.turretDeadZoneStartDeg))
          < Math.abs(getAngleDifference(desiredYaw360, TurretConstants.turretDeadZoneEndDeg))) {
        return TurretConstants.turretDeadZoneStartRot;
      } else {
        return TurretConstants.turretDeadZoneEndRot;
      }
    } else {
      return Rotation2d.fromDegrees(desiredYaw360);
    }
  }

  public void toggleManualFlywheel() {
    manualMode = !manualMode;
  }

  @Override
  public void periodic() {
    if (autoAimEnabled) {
      setTarget();
      aimAtTarget(currentTargetPose, isScoring);
    } else if (manualMode) {
      // When in manual shooting mode, turn on the flywheel when trigger is partially sequeezed
      // and set the hood to the max angle for close range shots
      io.setTurretPitch(TurretConstants.turretMaxHoodRot);
      flywheel.setFlywheelSpeed(TurretConstants.manualShotSpeedRpm);
    } else {
      io.setTurretPitch(TurretConstants.turretMaxHoodRot);
      flywheel.setFlywheelSpeed(0);
    }

    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret/Inputs", inputs);

    Pose2d pose =
        dtPose.get().plus(new Transform2d(TurretConstants.turretPosition, Rotation2d.kZero));
    Translation3d dtPos3d =
        new Translation3d(pose.getTranslation().getX(), pose.getTranslation().getY(), 0.5);
    turretVisual[0] = new Pose3d(dtPos3d, new Rotation3d());
    turretVisual[1] =
        new Pose3d(
            dtPos3d.plus(
                new Translation3d(
                    inputs.turretYaw.plus(pose.getRotation()).getCos() * 0.5,
                    inputs.turretYaw.plus(pose.getRotation()).getSin() * 0.5,
                    // TODO: this isn't accurate for high pitch angles
                    inputs.turretPitch.getSin() * 0.65)),
            new Rotation3d(dtPose.get().getRotation()));

    Logger.recordOutput("Shooter/Turret/Visual", turretVisual);
  }
}
