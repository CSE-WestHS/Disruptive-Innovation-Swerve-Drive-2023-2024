// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Rumble.RumbleForTime;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Rumble.Rumble;

public class AcquireNote extends Command {
  // private Rumble rumble;
  private Indexer indexer;
  private Intake intake;
  private double startTime;
  private Rumble rumble;
  /** Creates a new IndexIn. */
  public AcquireNote(Indexer Indexer, Intake Intake, Rumble Rumble) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake);
    this.rumble = Rumble;
    this.indexer = Indexer;
    this.intake = Intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    indexer.runVelocity(1200);
    intake.runVelocity(2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.runVelocity(1200);
    intake.runVelocity(2000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (indexer.getBeamState()) {
      new RumbleForTime(rumble, RumbleType.kBothRumble, 1, 0.25);
      return true;
    } else if ((Timer.getFPGATimestamp() - startTime) > 5) {
      return true;
    }
    return false;
  }
}
