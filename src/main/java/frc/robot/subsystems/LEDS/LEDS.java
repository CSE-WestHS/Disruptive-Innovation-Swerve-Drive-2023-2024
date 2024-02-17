// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDS {
  private AddressableLED leds;
  private AddressableLEDBuffer buffer;

  public LEDS() {
    leds = new AddressableLED(4);
    buffer = new AddressableLEDBuffer(frc.robot.Constants.LED_LENGTH);
    leds.setLength(frc.robot.Constants.LED_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  public void RunLEDS() {
    for (var i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for red
      buffer.setHSV(i, 0, 100, 50);
    }
    System.out.println("set LEDs");
    leds.setData(buffer);
  }

  private void solid(double percent, Color color) {
    for (int i = 0;
        i
            < MathUtil.clamp(
                frc.robot.Constants.LED_LENGTH * percent, 0, frc.robot.Constants.LED_LENGTH);
        i++) {
      buffer.setHSV(i, 100, 100, 50);
    }
  }
}
