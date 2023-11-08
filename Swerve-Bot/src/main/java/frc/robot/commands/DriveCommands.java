package frc.robot.commands;

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

    public Command JoyStickDriveCommand(){
        double x = OI.getLeftX();
        double y = OI.getLeftY();
        double rotation = OI.getRightX();
        return Commands.run(() -> swerve.drive(x, y, rotation), swerve);
    }
}
