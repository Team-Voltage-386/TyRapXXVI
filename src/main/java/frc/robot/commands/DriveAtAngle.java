// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAtAngle extends Command {
  private static final double ANGLE_KP = 12.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 300; // degrees per second
  private static final double ANGLE_MAX_ACCELERATION = 400; // degrees per second squared

  private final LoggedNetworkNumber driveAtAngleKp =
      new LoggedNetworkNumber("/Tuning/driveAtAnglekP", ANGLE_KP);
  private final LoggedNetworkNumber driveAtAngleMaxAngularVelocityDps =
      new LoggedNetworkNumber("/Tuning/driveAtAngleMaxAngularVelocityDps", ANGLE_MAX_VELOCITY);
  private final LoggedNetworkNumber driveAtAngleMaxAngularAccelerationDpss =
      new LoggedNetworkNumber(
          "/Tuning/driveAtAngleMaxAngularAccelerationDpss", ANGLE_MAX_ACCELERATION);

  private final ProfiledPIDController angleController;
  protected final Drive drive;
  protected DoubleSupplier xSupplier;
  protected DoubleSupplier ySupplier;
  protected Supplier<Rotation2d> rotationSupplier;

  /** Creates a new DriveAtAngle. */
  public DriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    // Create PID controller
    angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.reset(drive.getPose().getRotation().getRadians());
    angleController.setP(driveAtAngleKp.get());
    angleController.setConstraints(
        new TrapezoidProfile.Constraints(
            driveAtAngleMaxAngularVelocityDps.get(), driveAtAngleMaxAngularAccelerationDpss.get()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get linear velocity
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
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
