package frc.robot;

import java.util.logging.LogManager;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    private static XboxController driveController = new XboxController(0);

    public static double getLeftX(){
        System.out.println(driveController.getLeftX());
        return driveController.getLeftX();
    }

    public static double getLeftY(){
        System.out.println(driveController.getRightX());
        return driveController.getRightX();
    }

    public static double getRightX(){
        System.out.println(driveController.getLeftY());
        return driveController.getRightX();
    }

    public static double getRightY(){
        System.out.println(driveController.getRightY());
        return driveController.getRightY();
    }

}
