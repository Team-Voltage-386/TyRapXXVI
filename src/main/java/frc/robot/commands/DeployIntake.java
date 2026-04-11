// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.jr.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TuningUtil;

public class DeployIntake extends Command {
  /** Creates a new DeployIntake. */
  IntakeSubsystem intakeSubsystem;

  protected static final TuningUtil deployRate =
      new TuningUtil("Intake/deployRate", IntakeConstants.deployRate);
  protected double deployRateVal = deployRate.getValue();

  public DeployIntake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    deployRateVal = deployRate.getValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate the desired speed based on the current position
    double intakeDeployFraction =
        intakeSubsystem.getPosition() / IntakeConstants.EXTENDED_DEPLOY_POSITION;
    // Compute current speed so that average speed of full deploy achieves the desired deploy time
    // Max speed should be when intakeDeployFraction is 0, and speed should be 0 when
    // intakeDeployFraction is 1
    double desiredSpeedRotPerSec = (1 - intakeDeployFraction) * deployRateVal;
    // Calculate the increment for one 50Hz cycle. May need to apply a minimum speed to prevent
    // stalling near end of deployment
    double deployStep = desiredSpeedRotPerSec * 0.02;
    double newSetpoint = intakeSubsystem.getPosition() + deployStep;
    // Do not allow setpoint to go past the extended position
    intakeSubsystem.setSetpoint(
        MathUtil.clamp(newSetpoint, IntakeConstants.EXTENDED_DEPLOY_POSITION, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setSetpoint(IntakeConstants.EXTENDED_DEPLOY_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isDeployed();
  }
}
