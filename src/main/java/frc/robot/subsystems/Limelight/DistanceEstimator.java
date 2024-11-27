// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class DistanceEstimator {
    public double table() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double ty = table.getEntry("ty").getDouble(0);
        double Angle = 25;
        double leHight = 8.215;
        double goalHight = 80;
        double angleTogoal = Angle + ty;
        angleTogoal = Units.degreesToRadians(angleTogoal); 
        double distancetolimelightgoal = (goalHight - leHight) / Math.atan(angleTogoal);
        Logger.recordOutput("distance from goal", distancetolimelightgoal);
        return distancetolimelightgoal;


    }

}
