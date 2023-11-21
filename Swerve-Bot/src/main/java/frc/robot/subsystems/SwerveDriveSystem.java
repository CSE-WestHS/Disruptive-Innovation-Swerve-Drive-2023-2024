package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;
import swervelib.simulation.SwerveIMUSimulation;

public class SwerveDriveSystem extends SubsystemBase {
    SwerveDrive swerve;
    SwerveIMUSimulation swerveSim;
    public SwerveDriveSystem() throws IOException {
        swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
        swerve.resetEncoders();
        
        swerveSim = new SwerveIMUSimulation();
    }

    public void drive(double x, double y, double rotation) {
        SmartDashboard.putString("Robot Position module 1", swerve.getModules()[0].getPosition().toString());
        SmartDashboard.putString("Robot Position module 2", swerve.getModules()[1].getPosition().toString());
        SmartDashboard.putString("Robot Position module 3", swerve.getModules()[2].getPosition().toString());
        SmartDashboard.putString("Robot Position module 4", swerve.getModules()[3].getPosition().toString());
        SmartDashboard.putNumber("Robot Velocity module 1", swerve.getModules()[0].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("Robot Velocity module 2", swerve.getModules()[1].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("Robot Velocity module 3", swerve.getModules()[2].getAngleMotor().getVelocity());
        SmartDashboard.putNumber("Robot Velocity module 4", swerve.getModules()[3].getAngleMotor().getVelocity());
        Translation2d translation = new Translation2d(x, y);
        swerve.drive(translation, rotation, false, true);
        updateOdometry();
    }

    public void updateOdometry() {
        swerve.updateOdometry();
    }

    public Rotation2d getPitch() {
        return swerve.getPitch();
    }

    public Rotation2d getYaw() {
        return swerve.getYaw();
    }

    public Rotation2d getRoll() {
        return swerve.getRoll();
    }
    public Pose2d getPose(){
        return swerve.getPose();
    }

    public ChassisSpeeds getVelocity(){
        return swerve.getRobotVelocity();
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // drive(0, 0, 0);
    }

    @Override
    public void simulationPeriodic(){
        
    }
}
