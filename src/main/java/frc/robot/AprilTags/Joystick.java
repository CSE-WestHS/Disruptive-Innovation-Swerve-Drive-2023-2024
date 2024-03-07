package frc.robot.AprilTags;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Joystick implements RotationSource {
  @Override
  public double getR(double Heading) {
    return -MathUtil.applyDeadband(
        RobotContainer.getController().getRightX(), Constants.DRIVEDEADBAND);
  }
}
