package frc.robot;

import java.util.logging.LogManager;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    private static XboxController driveController = new XboxController(0);

    public static double getLeftX(){
        return driveController.getLeftX();
    }

    public static double getLeftY(){
        return driveController.getLeftY();
    }

    public static double getRightX(){
        return driveController.getRightX();
    }

    public static double getRightY(){
        return driveController.getRightY();
    }

}
