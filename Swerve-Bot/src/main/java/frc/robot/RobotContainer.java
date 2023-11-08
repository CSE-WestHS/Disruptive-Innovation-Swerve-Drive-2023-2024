// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.SwerveDriveSystem;

public class RobotContainer {
  SwerveDriveSystem swerve;
  DriveCommands driveCommands;
  public RobotContainer(){
    try{
    swerve = new SwerveDriveSystem();
    driveCommands = new DriveCommands(swerve);
    swerve.setDefaultCommand(driveCommands.JoyStickDriveCommand());
    }
    catch(IOException e){
      e.printStackTrace();
    }
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
