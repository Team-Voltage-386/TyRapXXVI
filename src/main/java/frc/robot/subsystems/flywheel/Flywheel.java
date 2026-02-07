package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  public final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;

    io.setFlywheelVelocity(100);
  }

  public Command shootCommand(Supplier<Double> RPM) {
    return new FunctionalCommand(
        () -> {},
        () -> io.setFlywheelVelocity(RPM.get()),
        (v) -> io.setFlywheelVelocity(0),
        () -> false,
        this);
  }

  public LinearVelocity getShotSpeed() {
    return inputs.shotSpeed;
  }

  public void setFlywheelSpeed(double rpm) {
    io.setFlywheelVelocity(rpm);
  }

  public double getFlywheelVelocity() {
    return io.getFlywheelVelocity();
  }

  @Override
  public void periodic() {
    io.readjustPID();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel/Inputs", inputs);
  }
}
