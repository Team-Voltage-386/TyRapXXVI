// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LightSubsystem;

/**
 * Will display whether the hub is active at a given time.
 */
public class HubActivity extends Command {
  private final Timer timer = new Timer();

  private LightSubsystem LightSubsystem;
  private CommandXboxController m_controller;
  private int counter = 0; // counter starts at 0
  private boolean hubIsActive = true;
  private boolean isAutoAhead =
      false; // Will be replaced by actual value when value is recieved during the match.

  // prerecorded times for hub activity. Each time the timer reaches the value in the array, the hub
  // toggles activity states.
  private final int[] timesWinning = {130, 105, 80, 55, -10};
  private final int[] timesLosing = {105, 80, 55, 30, -10};

  /**
   * Constructor for the CycleLED Command class.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HubActivity(LightSubsystem subsystem, CommandXboxController m_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LightSubsystem = subsystem;
    this.m_controller = m_controller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    counter = 0;
    isAutoAhead = false;
    System.out.println("Hub Scheduled");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isAutoAhead) {
      if (Timer.getMatchTime() < timesWinning[counter]) {
        counter++;
        hubIsActive = !hubIsActive;
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        System.out.println("Hub is " + hubIsActive);
      }
      if (Timer.getMatchTime() - 3 < timesWinning[counter]) {
        // Add rumble logic here
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.4);
      }
    } else {
      if (Timer.getMatchTime() < timesLosing[counter]) {
        counter++;
        hubIsActive = !hubIsActive;
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        System.out.println("Hub is " + hubIsActive);
      }
      if (Timer.getMatchTime() - 3 < timesLosing[counter]) {
        // Add rumble logic here
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.4);
      }
    }
    if (hubIsActive) {
      LightSubsystem.changeAllLEDColor(0, 255, 0);
    } else {
      LightSubsystem.changeAllLEDColor(255, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAutoAhead) {
      return counter >= timesWinning.length; // Returns true when the game ends.
    } else {
      return counter >= timesLosing.length;
    }
  }

  public boolean hubIsActive() {
    return hubIsActive;
  }

  public void setIsAhead(boolean setTo) {
    isAutoAhead = setTo;
  }
}
