// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class sets up the USB Camera and its MjpegServer. More documentation and methods can be
 * added here as needed.
 */
public class testTrack {
  private UsbCamera usbCamera;

  private UsbCamera setupServer(int id) {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    ((MjpegServer) CameraServer.getServer()).setCompression(30);
    return camera;
  }

  public void useCamera() {
    usbCamera = setupServer(0);
    usbCamera.setFPS(24);
    usbCamera.setResolution(320, 240);

    CameraServer.getServer().setSource(usbCamera);
    SmartDashboard.putString("Current Camera", "Front");
  }
}
