// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.jr.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TuningUtil;

public class CycleIntake extends Command {
  protected double currentSetPointDeg = 0;
  protected IntakeSubsystem intake;

  TuningUtil riseTimeSec =
      new TuningUtil("Tuning/Intake/CycleRiseTimeSec", IntakeConstants.riseTimeDefaultSec);
  TuningUtil fallTimeSec =
      new TuningUtil("Tuning/Intake/CycleFallTimeSec", IntakeConstants.fallTimeDefaultSec);
  TuningUtil maxAngleDeg =
      new TuningUtil("Tuning/Intake/CycleAngleDeg", IntakeConstants.maxCycleAngleDefaultDeg);

  protected double stepTimeSec = 0.02;
  protected int riseTimeSteps = 0;
  protected int fallTimeSteps = 0;
  protected double angleDeltaPerStepDeg = 0.0;
  protected double currentMaxAngleDeg = 0.0;

  /** Creates a new CycleIntake Command. */
  public CycleIntake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start at fully extended position
    currentSetPointDeg = IntakeConstants.extendedAngle.getDegrees();
    // Calcualte required degrees of motion per time step
    riseTimeSteps = (int) (riseTimeSec.getValue() / stepTimeSec);
    fallTimeSteps = (int) (fallTimeSec.getValue() / stepTimeSec);
    currentMaxAngleDeg =
        maxAngleDeg.getValue(); // Store so this doesn't change during command execution
    // Initialize velocity to rising portion of the cycle
    angleDeltaPerStepDeg = currentMaxAngleDeg / (double) riseTimeSteps;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentSetPointDeg += angleDeltaPerStepDeg;
    if (currentSetPointDeg > currentMaxAngleDeg) {
      // Transition to fall portion of cycle
      angleDeltaPerStepDeg = -1.0 * currentMaxAngleDeg / (double) fallTimeSteps;
      currentSetPointDeg += angleDeltaPerStepDeg;
    } else if (currentSetPointDeg < IntakeConstants.extendedAngle.getDegrees()) {
      // Transition to rise portion of cycle
      angleDeltaPerStepDeg = currentMaxAngleDeg / (double) riseTimeSteps;
      currentSetPointDeg += angleDeltaPerStepDeg;
    }
    intake.setAngle(Rotation2d.fromDegrees(currentSetPointDeg));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set to fully extended when command terminates
    this.intake.setAngle(IntakeConstants.extendedAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
