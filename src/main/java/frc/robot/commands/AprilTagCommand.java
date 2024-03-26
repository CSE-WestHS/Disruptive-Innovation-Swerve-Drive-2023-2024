// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTags.AprilTagLock;
import frc.robot.RobotContainer;

public class AprilTagCommand extends Command {
  private AprilTagLock apriltaglock;
  double time;
  /** Creates a new AprilTagCommadn. */
  public AprilTagCommand(AprilTagLock AprilTagLock) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.apriltaglock = AprilTagLock;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    frc.robot.RobotContainer.getHijack()
        .getR(apriltaglock.getR(frc.robot.RobotContainer.getAprilTagId()));
    time = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.getHijack().getR(4) - RobotContainer.getController().getRightX() < 25) {
      return true;
    }
    if (time > 5.0) {
      return true;
    }
    return false;
  }
}
