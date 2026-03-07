// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.jr.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeIOSim implements IntakeIO {

  private final IntakeSimulation intakeSimulation;
  protected boolean isDeployed = false;
  protected IntakingState intakingState = IntakingState.STOPPED;
  // the main mechanism object
  LoggedMechanism2d mech = new LoggedMechanism2d(0.7366, 0.3048);
  // the mechanism root node
  LoggedMechanismRoot2d root = mech.getRoot("Intake", 0.66, 0.1);
  LoggedMechanismLigament2d intakeMechanism;
  Rotation2d currentSetPoint = Rotation2d.fromDegrees(90);

  /** Creates a new SimIntakeSubsystem. */
  public IntakeIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
    intakeMechanism = root.append((new LoggedMechanismLigament2d("IntakeArm", 0.3, 90)));
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrainSimulation,
            // Width of the intake
            Inches.of(29),
            // The extension length of the intake beyond the robot's frame (when activated)
            Inches.of(12),
            // The intake is mounted on the front side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 40 fuel
            40);
    this.intakeSimulation.setGamePiecesCount(8);
  }

  public void deploy() {
    System.out.println("deploying intake mechanism");
    this.isDeployed = true;
    this.intakeSimulation.startIntake();
    currentSetPoint = IntakeConstants.extendedAngle;
  }

  public void retract() {
    System.out.println("retracting intake mechanism");
    this.isDeployed = false;
    this.intakeSimulation.stopIntake();
    currentSetPoint = IntakeConstants.retractedAngle;
  }

  public void takeIn() {
    System.out.println("taking in balls");
    this.intakingState = IntakingState.INTAKING;
  }

  public void stopMotor() {
    System.out.println("Stopping motor");
    this.intakingState = IntakingState.STOPPED;
  }

  public void reverse() {
    System.out.println("Reversing motor");
    this.intakingState = IntakingState.REVERSE;
  }

  public boolean isMotorStalled() {
    return false;
  }

  public int getBallCount() {
    return this.intakeSimulation.getGamePiecesAmount();
  }

  public int removeBall() {
    return this.intakeSimulation.setGamePiecesCount(
        this.intakeSimulation.getGamePiecesAmount() - 1);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = true;
    inputs.deployed = this.isDeployed;
    inputs.intakingState = this.intakingState;
    if (currentSetPoint == IntakeConstants.extendedAngle) {
      if (intakeMechanism.getAngle() > IntakeConstants.extendedAngle.getDegrees()) {
        intakeMechanism.setAngle(intakeMechanism.getAngle() - (IntakeConstants.speed / 50));
      }
    } else {
      if (intakeMechanism.getAngle() < IntakeConstants.retractedAngle.getDegrees()) {
        intakeMechanism.setAngle(intakeMechanism.getAngle() + (IntakeConstants.speed / 50));
      }
    }
    Logger.recordOutput("Intake/BallCount", this.intakeSimulation.getGamePiecesAmount());
    Logger.recordOutput("Intake/Mech", mech);
  }
}
