// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.SwerveDriveSystem;

public class RobotContainer {
  SwerveDriveSystem swerve;
  DriveCommands driveCommands;
  public RobotContainer(){
    try{
    swerve = new SwerveDriveSystem();
    driveCommands = new DriveCommands(swerve);
    Command joystickDrive = new JoystickDrive(swerve);
    swerve.setDefaultCommand(joystickDrive);
    }
    catch(IOException e){
      throw new RuntimeException(e);
    }
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
