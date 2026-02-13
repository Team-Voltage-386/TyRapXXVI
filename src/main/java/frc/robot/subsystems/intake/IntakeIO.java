// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  enum IntakingState {
    INTAKING,
    REVERSE,
    STOPPED
  }

  @AutoLog
  class IntakeIOInputs {
    public boolean connected = false;
    public boolean deployed = false;
    public IntakingState intakingState = IntakingState.STOPPED;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  public void deploy();

  public void retract();

  public void takeIn();

  public void stopMotor();

  public void reverse();

  public boolean isMotorStalled();
}
