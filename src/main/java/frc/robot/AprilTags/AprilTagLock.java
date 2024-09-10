package frc.robot.AprilTags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
// import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import org.littletonrobotics.junction.Logger;

public class AprilTagLock implements RotationSource {
  int speakerTarget;
  LimelightHelpers limelight = new LimelightHelpers();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private double PIDResult;
  public static PIDController rotationPID = new PIDController(1, 0, 0);

  public AprilTagLock(int target) {
    speakerTarget = target;
  }

  private static PIDController createPIDController() {
    rotationPID.setTolerance(10); // allowable angle error
    rotationPID.enableContinuousInput(
        -180, 180); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
    rotationPID.setSetpoint(0); // 0 = apriltag angle
    return rotationPID;
  }

  @Override
  public double getR(double Heading) {
    System.out.println("April Tag Calculations made!");
    String dump = limelight.getJSONDump("limelight");
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    PIDController pid = createPIDController();
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials =
        llresults.targetingResults.targets_Fiducials;
    SmartDashboard.putString("llresults", dump);
    LimelightTarget_Fiducial target = getMainId(fiducials, speakerTarget);
    if (target == null) {
      return pid.calculate(0);
    }
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      PIDResult = pid.calculate(-target.tx);
    }
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      PIDResult = pid.calculate(target.tx);
    }

    SmartDashboard.putNumber("tx", target.tx);
    Logger.recordOutput("Drive/R", PIDResult);

    return PIDResult;

    // return rotationPID.calculate(table.getEntry("tx").getDouble(0));

  }

  public LimelightHelpers.LimelightTarget_Fiducial getMainId(
      LimelightHelpers.LimelightTarget_Fiducial[] Fiducials, int id) {
    for (LimelightTarget_Fiducial fiducial : Fiducials) {
      if (fiducial.fiducialID == id) {
        return fiducial;
      }
    }
    return null;
  }
}
