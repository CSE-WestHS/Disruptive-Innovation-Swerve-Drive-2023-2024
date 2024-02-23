// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ArmSetAngle extends Command {
  private Arm arm;
  private double angle;

  /** Creates a new ArmSetAngle. */
  public ArmSetAngle(Arm Arm, double Angle) {
    this.arm = Arm;
    this.angle = Angle;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setPosition(angle);
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
    if (Math.abs(arm.getPosition() - angle) > 0.25) {
      return false;
    }
    return true;
    // return false;
  }
}
