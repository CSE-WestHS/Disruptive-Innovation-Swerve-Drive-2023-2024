package frc.robot.AprilTags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  public static PIDController rotationPID = createPIDController();

  public AprilTagLock(int target) {
    speakerTarget = target;
  }

  private static PIDController createPIDController() {
    PIDController pid = new PIDController(0.025, 0, 0);
    pid.setTolerance(1); // allowable angle error
    pid.enableContinuousInput(
        -180, 180); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
    pid.setSetpoint(0); // 0 = apriltag angle
    return pid;
  }

  @Override
  public double getR(double Heading) {
    String dump = limelight.getJSONDump("limelight");
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials =
        llresults.targetingResults.targets_Fiducials;
    SmartDashboard.putString("llresults", dump);
    LimelightTarget_Fiducial target = getMainId(fiducials, speakerTarget);
    if (target == null) {
      return rotationPID.calculate(0);
    }
    PIDResult = rotationPID.calculate(-target.tx);
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
