package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Pose3d[] turretVisual = new Pose3d[2];

  private final Supplier<Pose2d> dtPose;

  public Shooter(ShooterIO io, Supplier<Pose2d> dtPose) {
    this.io = io;
    this.dtPose = dtPose;

    io.setTurretPitch(Rotation2d.fromDegrees(45));
    io.setTurretYaw(Rotation2d.k180deg);
  }

  // TODO: cleanup API; extract these into individual commands and don't expose to outside
  public void addPitch(Rotation2d deltaPitch) {
    io.setTurretPitch(inputs.turretPitch.plus(deltaPitch));
  }

  public void addYaw(Rotation2d deltaYaw) {
    io.setTurretYaw(inputs.turretYaw.plus(deltaYaw));
  }

  public void shoot() {
    io.beginShooting();
  }

  public void stop() {
    io.endShooting();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Inputs", inputs);

    Translation3d dtPos =
        new Translation3d(
            dtPose.get().getTranslation().getX(), dtPose.get().getTranslation().getY(), 0.5);
    turretVisual[0] = new Pose3d(dtPos, new Rotation3d());
    turretVisual[1] =
        new Pose3d(
            dtPos.plus(
                new Translation3d(
                    inputs.turretYaw.getCos() * 0.5,
                    inputs.turretYaw.getSin() * 0.5,
                    inputs.turretPitch.getSin() * 0.5)),
            new Rotation3d());

    Logger.recordOutput("Shooter/Mechanism", turretVisual);
  }
}
