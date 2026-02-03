package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for turret (shooter aimer) hardware interaction. */
public interface TurretIO {

  @AutoLog
  class TurretIOInputs {

    public boolean connected = false;
    public Rotation2d turretYaw = new Rotation2d();
    public Rotation2d turretPitch = new Rotation2d();
  }

  default void updateInputs(TurretIOInputs inputs) {}

  /** Set the turret yaw to the specified position. */
  default void setTurretYaw(Rotation2d position) {}

  /* Set the turret pitch to the specified position. */
  default void setTurretPitch(Rotation2d position) {}
}
