// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;

/** Cycles the LEDs between red, green, and blue.
 *  Each color is held for 2 seconds. */
public class CycleLED extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LightSubsystem lightSubsystem;

  private final Timer timer = new Timer();
  private double lastRecordedTime;
  private int counter = 1; // counter starts at 1

  /**
   * Constructor for the CycleLED Command class.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CycleLED(LightSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lightSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    lastRecordedTime = timer.get();
    lightSubsystem.changeAllLEDColor(255, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() - lastRecordedTime >= 2) {
      cycleColor(counter);
      counter++;
      lastRecordedTime = timer.get();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    lightSubsystem.changeAllLEDColor(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*  For this style of command, isFinished never returns true.
    This is because the command is supposed to run until it is
    interrupted. The scheduler will automatically call end()
    once that happens. No need to worry about isFinished().
    */
    return false;
  }

  private void cycleColor(int counter) {

    if (counter % 3 == 0) {
      lightSubsystem.changeAllLEDColor(255, 0, 0); // Change this to whatever color you please.
    } else if (counter % 3 == 1) {
      lightSubsystem.changeAllLEDColor(0, 255, 0); // Change this to whatever color you please.
    } else {
      lightSubsystem.changeAllLEDColor(0, 0, 255); // Change this to whatever color you please.
    }
  }
}
