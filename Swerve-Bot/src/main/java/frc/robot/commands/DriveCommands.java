package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OI;
import frc.robot.subsystems.SwerveDriveSystem;
public class DriveCommands{

    private SwerveDriveSystem swerve;
    public DriveCommands(SwerveDriveSystem swerve){
        this.swerve = swerve;
    }

}
