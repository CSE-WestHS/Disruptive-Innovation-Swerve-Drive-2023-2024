// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class Limelight {
  static double Rotation(int target) {
    String Dumb = LimelightHelpers.getJSONDump("");
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("");
    LimelightHelpers.LimelightTarget_Fiducial[] april = results.targetingResults.targets_Fiducials;
    SmartDashboard.putString("results", Dumb);
    LimelightTarget_Fiducial targetF = getId(april, target);
    if (targetF == null) {
      return 0;
    } else {
      return targetF.tx;
    }
  }

  static LimelightHelpers.LimelightTarget_Fiducial getId(
      LimelightHelpers.LimelightTarget_Fiducial[] uno, int dos) {
    for (LimelightTarget_Fiducial april : uno) {
      if (april.fiducialID == dos) {
        return april;
      }
    }
    return null;
  }
}
