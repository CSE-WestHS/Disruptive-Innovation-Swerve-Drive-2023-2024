// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmUpGradual extends Command {
  private Arm arm;
  private double angle = frc.robot.Constants.ANGLE_SPEAKER;

  /** Creates a new ArmUpGradual. */
  public ArmUpGradual(Arm Arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = Arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 1300 = speed

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPosition(angle);
    angle += (1 / 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPosition(frc.robot.Constants.ANGLE_SPEAKER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (angle > 150) {
      return true;
    }
    return false;
  }
}
