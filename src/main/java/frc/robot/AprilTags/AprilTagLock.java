package frc.robot.AprilTags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class AprilTagLock implements RotationSource {
  LimelightHelpers limelight = new LimelightHelpers();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private double PIDResult;
  public static PIDController rotationPID = createPIDController();

  private static PIDController createPIDController() {
    PIDController pid = new PIDController(0.75, .01, 0);
    pid.setTolerance(.25); // allowable angle error
    pid.enableContinuousInput(
        -180, 180); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
    pid.setSetpoint(0); // 0 = apriltag angle
    return pid;
  }

  @Override
  public double getR(double Heading) {
    PIDResult = rotationPID.calculate(-table.getEntry("tx").getDouble(0));
    Logger.recordOutput("Drive/R", PIDResult);
    String dump = limelight.getJSONDump("limelight");
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    LimelightHelpers.LimelightTarget_Fiducial[] fiducials =
        llresults.targetingResults.targets_Fiducials;
    // Logger.recordOutput("fiducials", fiducials);
    SmartDashboard.putString("llresults", dump);

    return PIDResult;

    // return rotationPID.calculate(table.getEntry("tx").getDouble(0));

  }
}
