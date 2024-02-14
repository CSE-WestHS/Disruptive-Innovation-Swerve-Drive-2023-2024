// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmDownGradual extends Command {
  private Arm arm;
  private double angle = arm.getPosition();

  /** Creates a new ArmUpGradual. */
  public ArmDownGradual(Arm Arm) {
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
    angle -= 0.1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPosition(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (angle < 56) {
      return true;
    }
    return false;
  }
}
