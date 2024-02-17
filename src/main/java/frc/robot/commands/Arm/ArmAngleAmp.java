// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmAngleAmp extends Command {
  private Arm arm;

  /** Creates a new ArmAngleAmp. */
  public ArmAngleAmp(Arm Arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = Arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setPosition(frc.robot.Constants.ANGLE_AMP);
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
    if (arm.getPosition() - frc.robot.Constants.ANGLE_AMP > 2) {
      return false;
    }
    return true;
  }
}
