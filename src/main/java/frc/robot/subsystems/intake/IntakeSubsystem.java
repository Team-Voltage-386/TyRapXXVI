package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO intakeIO;
  private final IntakeIO.IntakeIOInputs inputs;

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    this.inputs = new IntakeIO.IntakeIOInputs();
  }

  public Command deployCommand() {
    return Commands.runOnce(() -> intakeIO.deploy());
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> intakeIO.retract());
  }

  public Command takeInCommand() {
    return Commands.runOnce(() -> intakeIO.takeIn());
  }

  public Command stopMotorCommand() {
    return Commands.runOnce(() -> intakeIO.stopMotor());
  }

  public Command reverseCommand() {
    return Commands.runOnce(() -> intakeIO.reverse());
  }

  public boolean isMotorStalled() {
    return intakeIO.isMotorStalled();
  }

  public void testDeployVoltage(double voltage) {
    intakeIO.testDeployVoltage(voltage);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(this.inputs);
  }
}
