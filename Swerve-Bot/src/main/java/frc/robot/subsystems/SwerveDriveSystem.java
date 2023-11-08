package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveDriveSystem extends SubsystemBase {
    SwerveDrive swerve;

    public SwerveDriveSystem() throws IOException{
        swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
        swerve.resetEncoders();

    }

    public void drive(double x, double y, double rotation){
        Translation2d translation = new Translation2d(x, y);
        swerve.drive(translation, rotation, false, true);
    }

    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      drive(0, 0, 0);
    }
}
