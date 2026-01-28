package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.function.BooleanConsumer;
import java.util.function.Consumer;

public class FlywheelIOSim implements FlywheelIO {

  private AngularVelocity flywheelSpeed = RPM.zero();

  private final Consumer<AngularVelocity> flywheelSpeedConsumer;
  private final BooleanConsumer flywheelShootingConsumer;

  public FlywheelIOSim(
      Consumer<AngularVelocity> flywheelSpeedConsumer, BooleanConsumer flywheelShootingConsumer) {
    this.flywheelSpeedConsumer = flywheelSpeedConsumer;
    this.flywheelShootingConsumer = flywheelShootingConsumer;
  }

  public void updateInputs(FlywheelIO.FlywheelIOInputs inputs) {
    inputs.connected = true;
    inputs.flywheelSpeed = flywheelSpeed;
    // assuming 12 meters per second at 6000 RPM
    inputs.shotSpeed = MetersPerSecond.of(flywheelSpeed.in(RPM) / 6000 * 12);
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelSpeed = speed;
    flywheelSpeedConsumer.accept(speed);
  }

  /**
   * Start/stop shooting Fuel with the current shooter configuration.
   */
  @Override
  public void setFlywheelShooting(boolean shooting) {
    flywheelShootingConsumer.accept(shooting);
  }
}
