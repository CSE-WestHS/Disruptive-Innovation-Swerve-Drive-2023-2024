// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;

public class AcquireNote extends Command {
  private Indexer indexer;
  private Intake intake;
  private double startTime;
  /** Creates a new IndexIn. */
  public AcquireNote(Indexer Indexer,Intake Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake);
    this.indexer = Indexer;
    this.intake = Intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    indexer.runVelocity(2000);
    intake.runVelocity(2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (indexer.getBeamState() || (Timer.getFPGATimestamp() - startTime) > 30) {
        return true;
    }
    return false;
  }
}
