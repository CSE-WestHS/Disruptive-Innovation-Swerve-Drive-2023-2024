// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDS {
  private AddressableLED leds;
  private AddressableLEDBuffer buffer;
  private static boolean isAuto = DriverStation.isAutonomous();
  private static Optional<edu.wpi.first.wpilibj.DriverStation.Alliance> Alliance = DriverStation.getAlliance();
  private static boolean isTeleop = DriverStation.isTeleop();

  public LEDS() {
    leds = new AddressableLED(4);
    buffer = new AddressableLEDBuffer(frc.robot.Constants.LED_LENGTH);
    leds.setLength(frc.robot.Constants.LED_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  public void RunLEDS() {
    if (DriverStation.isDisabled()) {
      for (var i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for yellow
        buffer.setHSV(i, 60, 100, 0);
      }
      System.out.println("set LEDs");
      leds.setData(buffer);
    }
    if (isAuto) {
      for (var i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        buffer.setHSV(i, 0, 100, 50);
      }
      System.out.println("set LEDs");
      leds.setData(buffer);
    }
    if (isTeleop) {
      for (var i = 0; i < buffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for some color
        buffer.setHSV(i, 75, 60, 50);
      }
      System.out.println("set LEDs");
      leds.setData(buffer);
    }
    
    //example
    // for (var i = 0; i < buffer.getLength(); i++) {
    //   // Sets the specified LED to the HSV values for red
    //   buffer.setHSV(i, 0, 100, 50);
    // }
    // System.out.println("set LEDs");
    // leds.setData(buffer);
    
  }
}
