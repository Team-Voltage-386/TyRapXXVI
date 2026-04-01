// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Makes the LEDs sequentially change to show a sort of rainbow effect. Currently works with 7 LEDs.
 * Goal is to make it work with the entire strip of LEDs available.
 */
public class JiggleIntake extends Command {

  private final Timer timer = new Timer();
  private final IntakeSubsystem intake;
  private double sign = 1;

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
      sign *= -1;
      timer.reset();
    }
    intake.testDeployVoltage(sign * 3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.testDeployVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
