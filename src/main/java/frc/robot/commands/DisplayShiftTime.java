// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Will display whether the hub is active at a given time.
 */
public class DisplayShiftTime extends Command {
  // Define the enum
  public enum GameStates {
    AUTO(0, 20),
    ALL(130, 10),
    SHIFT1(105, 25),
    SHIFT2(80, 25),
    SHIFT3(55, 25),
    SHIFT4(30, 25),
    ENDGAME(0, 30),
    DONE(-1, 0);

    private final int endTime;
    private final double shiftTime;

    // GameStates Constructor
    GameStates(int endTime, double shiftTime) {
      this.endTime = endTime;
      this.shiftTime = shiftTime;
    }

    public int getEndTime() {
      return endTime;
    }

    public double getShiftTime() {
      return shiftTime;
    }
  }

  private GameStates thisGameState = GameStates.ALL;
  private double shiftTimeLeft = 10;

  /**
   * Constructor
   *
   */
  public DisplayShiftTime() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("GameShift/CurrentShift", GameStates.ALL.name());
    Logger.recordOutput("GameShift/shiftTimeLeft", GameStates.ALL.getShiftTime());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (thisGameState) {
      case AUTO:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          thisGameState = GameStates.ALL;
        }
        break;
      case ALL:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          thisGameState = GameStates.SHIFT1;
        }
        break;
      case SHIFT1:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          thisGameState = GameStates.SHIFT2;
        }
        break;
      case SHIFT2:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          thisGameState = GameStates.SHIFT3;
        }
        break;
      case SHIFT3:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          thisGameState = GameStates.SHIFT4;
        }
        break;
      case SHIFT4:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          thisGameState = GameStates.ENDGAME;
        }
        break;
      case ENDGAME:
        if (Timer.getMatchTime() == thisGameState.getEndTime()) {
          thisGameState = GameStates.DONE;
        }
        break;
        //Note: "DONE" does not show up in Sim
      case DONE:
        System.out.println("Change Shift: Done");
        break;
      default:
        break;
    }
    shiftTimeLeft = Timer.getMatchTime() - thisGameState.getEndTime();
    Logger.recordOutput("GameShift/CurrentShift", thisGameState.name());
    Logger.recordOutput("GameShift/shiftTimeLeft", shiftTimeLeft);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
