// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.commands.Arm.ArmAngleSpeaker;
// import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
// import frc.robot.commands.Arm.*;

public class ShootNoteSpeaker extends Command {
  private Indexer indexer;
  private Shooter shooter;
  private double targetSpeed;
  private double startTime;

  /**
   * Creates a new ShootNote.
   *
   * @param d
   */
  public ShootNoteSpeaker(Indexer Indexer, Shooter Shooter, double TargetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = Indexer;
    this.shooter = Shooter;
    this.targetSpeed = TargetSpeed;
    addRequirements(shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shooter.runVelocity(targetSpeed);
    System.out.println("start rollers");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runVelocity(targetSpeed);
    if (Math.abs(shooter.getVelocityRPM() - targetSpeed) < 100) {

      indexer.runVelocity(2000);
      System.out.println("start indexer");
    }
    System.out.println("execute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(
        "end******************************************************************************");
    indexer.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!indexer.getBeamState() || (Timer.getFPGATimestamp() - startTime) > 3) {
      return true;
    }
    return false;
  }
}
