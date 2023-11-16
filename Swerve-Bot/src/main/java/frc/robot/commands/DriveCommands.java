package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
        double x = MathUtil.applyDeadband(OI.getLeftX(), 0.2)  * 3;
        double y = MathUtil.applyDeadband(OI.getLeftY(), 0.2) * 3;
        double rotation = MathUtil.applyDeadband(OI.getRightX(), 0.2);
        return Commands.run(() -> swerve.drive(x, y, rotation), swerve);
    }
}
