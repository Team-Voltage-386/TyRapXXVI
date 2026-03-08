package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TuningUtil;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  public final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  TuningUtil setRPM = new TuningUtil("/Tuning/Flywheel/TestSetRPM", 2200);

  public Flywheel(FlywheelIO io) {
    this.io = io;

    io.setFlywheelVelocity(0);
  }

  public Command shootCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> io.setFlywheelVelocity(setRPM.getValue()),
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

  public void testFlywheelVoltage(double volts) {
    io.testFlywheelVoltage(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel/Inputs", inputs);
  }
}
