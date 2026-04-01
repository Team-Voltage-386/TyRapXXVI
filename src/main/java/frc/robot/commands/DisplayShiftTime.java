// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Will display whether the hub is active at a given time.
 */
public class DisplayShiftTime extends Command {
  // Define the enum
  public enum GameStates {
    INIT(-1, -1, -1),
    ALL(130, 10, 5),
    ALL_SHIFT1(105, 35, 5),
    SHIFT1(105, 25, 5),
    SHIFT2(80, 25, 5),
    SHIFT3(55, 25, 5),
    SHIFT4(30, 25, 5),
    SHIFT4_ENDGAME(0, 55, 5),
    ENDGAME(0, 30, 5),
    DONE(-1, 0, 0);

    private final int endTime;
    private final int shiftTime;
    private final int warnSeconds;

    // GameStates Constructor
    GameStates(int endTime, int shiftTime, int warnSeconds) {
      this.endTime = endTime;
      this.shiftTime = shiftTime;
      this.warnSeconds = warnSeconds;
    }

    // Get the end time for the shift
    public int getEndTime() {
      return endTime;
    }

    // Get the total shift time
    public int getShiftTime() {
      return shiftTime;
    }

    // Get the warning time in seconds
    public int getWarnSeconds() {
      return warnSeconds;
    }
  }

  private GameStates thisGameState = GameStates.INIT;
  private int shiftTimeLeft = 10;
  private final BooleanSupplier hubIsAheadSup;
  private boolean okToShoot = false;
  private boolean warnLight = false;
  private boolean timerToggle = false;
  private final Timer warnTimer = new Timer();

  /**
   * Constructor
   *
   */
  public DisplayShiftTime(BooleanSupplier hubIsAheadSup) {
    this.hubIsAheadSup = hubIsAheadSup;
    Logger.recordOutput("GameShift/CurrentShift", GameStates.INIT.name());
    Logger.recordOutput("GameShift/shiftTimeLeft", GameStates.INIT.getShiftTime());
    Logger.recordOutput("GameShift/hubIsAhead", this.hubIsAheadSup.getAsBoolean());
    Logger.recordOutput("GameShift/okToShoot", okToShoot);
    Logger.recordOutput("GameShift/warnLight", warnLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start the time for warnings
    warnTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Toggle the warning signal and reset the warning timer
    if (warnTimer.hasElapsed(0.25)) {
      timerToggle = !timerToggle;
      warnTimer.reset();
    }

    // Display warning timer
    if (((int) Timer.getMatchTime() - thisGameState.getEndTime())
        <= thisGameState.getWarnSeconds()) {
      warnLight = timerToggle;
    } else {
      warnLight = false;
    }

    // Move through the game states
    switch (thisGameState) {
      // Initial state
      case INIT:
        if (DriverStation.isEnabled() && !DriverStation.isAutonomous()) {
          if (hubIsAheadSup.getAsBoolean()) {
            this.thisGameState = GameStates.ALL_SHIFT1;
          } else {
            thisGameState = GameStates.ALL;
          }
        }
        break;
      // ALL Shooting
      case ALL:
        okToShoot = true;
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = !okToShoot;
          thisGameState = GameStates.SHIFT1;
        }
        break;
      // Combined ALL and SHIFT1
      case ALL_SHIFT1:
        okToShoot = true;

        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = !okToShoot;
          thisGameState = GameStates.SHIFT2;
        }
        break;
      // Only SHIFT1
      case SHIFT1:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = !okToShoot;
          thisGameState = GameStates.SHIFT2;
        }
        break;
      // Only SHIFT2
      case SHIFT2:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = !okToShoot;
          thisGameState = GameStates.SHIFT3;
        }
        break;
      // Only SHIFT3
      case SHIFT3:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = !okToShoot;
          if (hubIsAheadSup.getAsBoolean()) {
            thisGameState = GameStates.SHIFT4;
          } else {
            thisGameState = GameStates.SHIFT4_ENDGAME;
          }
        }
        break;
      // Only SHIFT4
      case SHIFT4:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = !okToShoot;
          thisGameState = GameStates.ENDGAME;
        }
        break;
      // Combined SHIFT4 and ENDGAME
      case SHIFT4_ENDGAME:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = false;
          thisGameState = GameStates.DONE;
        }
        break;
      // Only ENDGAME
      case ENDGAME:
        if (Timer.getMatchTime() <= thisGameState.getEndTime()) {
          okToShoot = false;
          thisGameState = GameStates.DONE;
        }
        break;
      // Note: "DONE" does not show up in Sim
      // DONE State
      case DONE:
        okToShoot = false;
        System.out.println("Change Shift: Done");
        break;
      // Default case is empty
      default:
        break;
    }

    // Calculate time left on the shift
    shiftTimeLeft = (int) Timer.getMatchTime() - thisGameState.getEndTime();

    // Display items on dashboard
    Logger.recordOutput("GameShift/CurrentShift", thisGameState.name());
    Logger.recordOutput("GameShift/shiftTimeLeft", shiftTimeLeft);
    Logger.recordOutput("GameShift/hubIsAhead", hubIsAheadSup.getAsBoolean());
    Logger.recordOutput("GameShift/okToShoot", okToShoot);
    Logger.recordOutput("GameShift/warnLight", warnLight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DisplayShiftTime: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command once the match is done
    if (DriverStation.getMatchTime() < 0) {
      System.out.println("DisplayShiftTime: isFinished");
      return true;
    } else {
      return false;
    }
  }
}
