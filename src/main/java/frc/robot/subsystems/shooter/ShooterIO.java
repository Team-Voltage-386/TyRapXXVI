package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {

    public boolean connected = false;
    public Rotation2d turretYaw = new Rotation2d();
    public Rotation2d turretPitch = new Rotation2d();
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Set the turret yaw to the specified position. */
  default void setTurretYaw(Rotation2d position) {}

  /* Set the turret pitch to the specified position. */
  default void setTurretPitch(Rotation2d position) {}

  /**
   * Begin shooting Fuel with the current shooter configuration.
   */
  default void beginShooting() {}

  /**
   * Stop shooting Fuel.
   */
  default void endShooting() {}
}
