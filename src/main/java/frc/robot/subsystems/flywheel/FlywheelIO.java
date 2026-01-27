package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/** Interface for shooter hardware interaction. */
public interface FlywheelIO {

  @AutoLog
  class FlywheelIOInputs {

    public boolean connected = false;
    public AngularVelocity flywheelSpeed = RPM.of(0);
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  /**
   * Set the turret wheel speed to the specified value. This does not start shooting; it only sets
   * the target speed.
   */
  default void setFlywheelSpeed(AngularVelocity speed) {}

  /**
   * Start/stop shooting Fuel with the current shooter configuration.
   */
  default void setFlywheelShooting(boolean shooting) {}
}
