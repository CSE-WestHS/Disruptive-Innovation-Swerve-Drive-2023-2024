// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.PID;
/** Add your docs here. */
public class AprilTagAiming {
    private static double PIDResult;
    private static int speakerTarget;
    private static PIDController aprilController = PID.apriltagPID;
    public static double getR(int Atarget) {
        speakerTarget = Atarget;
        System.out.println("April Tag Calculations made!");
        String dump = LimelightHelpers.getJSONDump("limelight");
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
        PIDController pid = aprilController;
        LimelightHelpers.LimelightTarget_Fiducial[] fiducials =
            llresults.targetingResults.targets_Fiducials;
        SmartDashboard.putString("llresults", dump);
        LimelightTarget_Fiducial target = getMainId(fiducials, speakerTarget);
        if (target == null) {
        return pid.calculate(0);
        } else {
            PIDResult = pid.calculate(target.tx);
        }

        SmartDashboard.putNumber("tx", target.tx);
        Logger.recordOutput("Drive/R", PIDResult);

        return PIDResult;
    }
    public static LimelightHelpers.LimelightTarget_Fiducial getMainId(
        LimelightHelpers.LimelightTarget_Fiducial[] Fiducials, int id) {
      for (LimelightTarget_Fiducial fiducial : Fiducials) {
        if (fiducial.fiducialID == id) {
          return fiducial;
        }
      }
      return null;
    }
}
