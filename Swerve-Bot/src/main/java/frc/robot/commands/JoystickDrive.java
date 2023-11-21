// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDriveSystem;

public class JoystickDrive extends CommandBase {
  /** Creates a new JoystickDrive. */
  private SwerveDriveSystem swerve;
  public JoystickDrive(SwerveDriveSystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
        this.swerve = swerve;
        addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = MathUtil.applyDeadband(OI.getLeftX(), 0.2)  * 3;
    double y = MathUtil.applyDeadband(OI.getLeftY(), 0.2) * 3;
    double rotation = MathUtil.applyDeadband(OI.getRightX(), 0.2);
    String input = Double.toString(x) + " " + Double.toString(y) + " " + Double.toString(rotation);
    SmartDashboard.putString("x y r", input);
    swerve.drive(x, y, rotation);
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
