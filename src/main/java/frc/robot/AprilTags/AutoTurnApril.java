// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AprilTags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagAiming;
import frc.robot.PID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class AutoTurnApril extends Command {
  /** Creates a new AutoTurnApril. */
  public Drive drive;
  private boolean isFlipped = false;
  private PIDController aprilcController;
  public AutoTurnApril(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.aprilcController = PID.apriltagPID;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  0,
                  0,
                  // omega * MAXSPEED_OMEGA
                  Math.toRadians(AprilTagAiming.getR(4)),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aprilcController.atSetpoint();
  }
}
