// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends DriveAtAngle {
  final Rotation2d threshold;

  /** Creates a new RotateToAngle. */
  public RotateToAngle(Drive drive, Supplier<Rotation2d> rotationSupplier, Rotation2d threshold) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drive, () -> 0, () -> 0, rotationSupplier);
    this.threshold = threshold;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d error = drive.getPose().getRotation().minus(rotationSupplier.get());
    return Math.abs(error.getDegrees()) <= threshold.getDegrees();
  }
}
