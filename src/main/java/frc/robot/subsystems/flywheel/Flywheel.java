package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;

    io.setFlywheelSpeed(RPM.of(4000));
  }

  public Command shootCommand() {
    return new FunctionalCommand(
        () -> {},
        () -> io.setFlywheelShooting(true),
        (v) -> io.setFlywheelShooting(false),
        () -> false,
        this);
  }

  public LinearVelocity getShotSpeed() {
    return inputs.shotSpeed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel/Inputs", inputs);
  }
}
