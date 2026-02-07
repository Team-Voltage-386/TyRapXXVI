// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;

/** Changes the color of a specific LED, given an index. */
public class ChangeLEDColorCommand extends Command {
  private LightSubsystem lightSubsystem;

  private int Index;
  private int R;
  private int G;
  private int B;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param Index The index of the led being commanded.
   * @param R The red value (0 to 255)
   * @param G The green value (0 to 255)
   * @param B The blue value (0 to 255)
   */
  public ChangeLEDColorCommand(LightSubsystem subsystem, int index, int r, int g, int b) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.lightSubsystem = subsystem;
    this.Index = index;
    this.R = r;
    this.G = g;
    this.B = b;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightSubsystem.setToColor(Index, R, G, B);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
