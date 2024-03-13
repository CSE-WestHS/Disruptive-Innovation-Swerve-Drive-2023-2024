// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIOSparkMax;

public class ZeroArm extends Command {
  private Arm arm;

  /** Creates a new ZeroArm. */
  public ZeroArm(Arm Arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = Arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmIOSparkMax.getEncoder().setPosition(frc.robot.Constants.ANGLE_START_POSITION);
    arm.setPosition(-Constants.ANGLE_SPEAKER);
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
