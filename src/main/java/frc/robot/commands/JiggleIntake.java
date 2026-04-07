// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Makes the LEDs sequentially change to show a sort of rainbow effect. Currently works with 7 LEDs.
 * Goal is to make it work with the entire strip of LEDs available.
 */
public class JiggleIntake extends Command {

  private final Timer timer = new Timer();
  private final IntakeSubsystem intake;
  private boolean out = true;

  public JiggleIntake(IntakeSubsystem subsystem) {
    intake = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > .66) {
      double voltage = out ? 3 : -1.5;
      if (intake.getPosition() > -5.0) {
        if (voltage > 0) {
          voltage = 0;
        }
      }
      intake.testDeployVoltage(voltage);
      out = !out;
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(intake.deployCommand());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
