package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class PID {
    public static PIDController apriltagPID = getAprilRotationPID();
    private static PIDController getAprilRotationPID() {
        //new PIDController(0.035, 0.001, 0)
        PIDController pid = new PIDController(
                0.035,
                0.001,
                0);
        pid.setTolerance(3);
        pid.enableContinuousInput(-180, 180); 
        pid.setSetpoint(0);
        return pid;
    }

}
