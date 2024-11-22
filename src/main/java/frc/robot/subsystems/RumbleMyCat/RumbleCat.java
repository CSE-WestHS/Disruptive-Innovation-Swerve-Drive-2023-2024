// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.RumbleMyCat;

// import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class RumbleCat {
  XboxController linuxs = new XboxController(0);

  public void setVibration(double value, RumbleType type) {
    linuxs.setRumble(type, value);
  }

  public void setVibration(double value) {
    setVibration(value, RumbleType.kBothRumble);
  }
}
