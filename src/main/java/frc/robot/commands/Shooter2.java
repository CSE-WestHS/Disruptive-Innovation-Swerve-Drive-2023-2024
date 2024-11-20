// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lowtapershooter.lowtapershooter;

public class Shooter2 extends Command {
  lowtapershooter shooter;
  double secondSecond;
  /** Creates a new Shooter2. */
  public Shooter2(lowtapershooter shooter3) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter3;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    secondSecond = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runVelocity(2500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Timer.getFPGATimestamp() - secondSecond) == 10) {
      return true;
    }
    return false;
  }
}
