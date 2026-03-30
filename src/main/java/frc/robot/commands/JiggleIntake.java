// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.jr.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Makes the LEDs sequentially change to show a sort of rainbow effect. Currently works with 7 LEDs.
 * Goal is to make it work with the entire strip of LEDs available.
 */
public class JiggleIntake extends Command {

  private final Timer timer = new Timer();
  private final IntakeSubsystem intake;
  private Command activeCommand = Commands.none();
  private double outSetpoint = IntakeConstants.EXTENDED_DEPLOY_POSITION;
  private double inSetpoint = IntakeConstants.HALF_DEPLOY_POSITION;

  public JiggleIntake(IntakeSubsystem subsystem) {
    intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    activeCommand.cancel();
    timer.reset();
    timer.start();
  }

  /**
   * Calculates a number between two numbers at a specific increment.
   *
   * @param start The start value.
   * @param end The end value.
   * @param amount The amount to interpolate between the two values (0.0 to 1.0).
   * @return The interpolated float value.
   */
  public static double lerp(double start, double end, double amount) {
    return start + (end - start) * amount;
    // An alternative, more precise method for floating point values is:
    // return (1 - amount) * start + amount * end;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double timeSec = timer.get() % 2;
    activeCommand.cancel();
    activeCommand =
        intake.setSetpoint(
            lerp(
                timeSec > 1 ? inSetpoint : outSetpoint,
                timeSec > 1 ? outSetpoint : inSetpoint,
                timer.get() % 1));
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    activeCommand.cancel();
    CommandScheduler.getInstance().schedule(intake.deployCommand());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
