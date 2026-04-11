// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.jr.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RetractIntake extends Command {
  protected double currentSetpoint;
  protected double targetSetpoint;
  IntakeSubsystem intakeSubsystem;

  /** Creates a new RetractIntake. */
  public RetractIntake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentSetpoint = intakeSubsystem.getPosition();
    targetSetpoint = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (currentSetpoint < targetSetpoint) {
      double deployInc =
          (currentSetpoint > targetSetpoint - IntakeConstants.deploySlowdownPointUp)
              ? IntakeConstants.deployIncPerStepSlow
              : IntakeConstants.deployIncPerStepFast;
      currentSetpoint = Math.min(targetSetpoint, currentSetpoint - deployInc);
    } else if (currentSetpoint > targetSetpoint) {
      double deployInc =
          (currentSetpoint < targetSetpoint + IntakeConstants.deploySlowdownPointDown)
              ? IntakeConstants.deployIncPerStepSlow
              : IntakeConstants.deployIncPerStepFast;
      currentSetpoint = Math.max(targetSetpoint, currentSetpoint + deployInc);
    }
    intakeSubsystem.setSetpoint(
        MathUtil.clamp(currentSetpoint, IntakeConstants.EXTENDED_DEPLOY_POSITION, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(currentSetpoint - targetSetpoint) < 0.001) {
      intakeSubsystem.setSetpoint(targetSetpoint);
      return true;
    }

    return false;
  }
}
