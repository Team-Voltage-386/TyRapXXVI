// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;

/** Makes the LEDs sequentially change to show a sort of rainbow effect.
 * Currently works with 7 LEDs.
 * Goal is to make it work with the entire strip of LEDs available.
 */
public class RainbowLED extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Timer timer = new Timer();

  double lastRecordedTime;
  private int counter = 0;
  private int hold = 0;
  private LightSubsystem lightSubsystem;
  private int[][] rainbow = {
    {255, 0, 0}, {255, 127, 0}, {255, 255, 0}, {0, 255, 0}, {0, 0, 255}, {75, 0, 130}, {148, 0, 211}
  };

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RainbowLED(LightSubsystem subsystem) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() - lastRecordedTime >= 1) {
      hold++;
      for (int i = 0; i < 76; i++) {
        lightSubsystem.setToColor(
            i, rainbow[counter % 7][0], rainbow[counter % 7][1], rainbow[counter % 7][2]);
        counter++;
      }
      counter = hold;
      lastRecordedTime = timer.get();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lightSubsystem.changeAllLEDColor(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
