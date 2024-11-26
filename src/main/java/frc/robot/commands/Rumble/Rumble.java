// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rumble;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RumbleMyCat.RumbleCat;

public class Rumble extends Command {
  RumbleCat rumble;
  double value = 0;
  /** Creates a new Rumble. */
  public Rumble(RumbleCat rumble, double value) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rumble = rumble;
    this.value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rumble.setVibration(value);
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
