// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Rumble;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends SubsystemBase {
  XboxController controller1;
  XboxController controller2;
  
  /** Creates a new Vibrator. */
  public Rumble(XboxController controller1, XboxController controller2) {
    this.controller1 = controller1;
    this.controller2 = controller2;
  }

  public void setVibration(double value, RumbleType type){
    controller1.setRumble(type, value);
    controller2.setRumble(type, value);
   }

  public void setVibration(double value){
    setVibration(value, RumbleType.kBothRumble);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}