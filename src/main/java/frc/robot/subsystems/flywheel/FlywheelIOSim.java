package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

public class FlywheelIOSim implements FlywheelIO {

  private AngularVelocity flywheelSpeed = RPM.zero();

  public FlywheelIOSim() {}

  public void updateInputs(FlywheelIO.FlywheelIOInputs inputs) {
    inputs.connected = true;
    inputs.flywheelSpeed = flywheelSpeed;
    // assuming 12 meters per second at 6000 RPM
    inputs.shotSpeed = MetersPerSecond.of(flywheelSpeed.in(RPM) / 6000 * 12);
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelSpeed = RPM.of(rpm);
    Logger.recordOutput("/Shooter/Flywheel/VelocitySetpoint", rpm);
  }

  @Override
  public double getFlywheelVelocity() {
    return flywheelSpeed.in(RPM);
  }
}
