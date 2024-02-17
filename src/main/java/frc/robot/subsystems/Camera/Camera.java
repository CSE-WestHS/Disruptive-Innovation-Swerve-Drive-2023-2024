// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Camera extends SubsystemBase {
  // Creates UsbCamera and MjpegServer [1] and connects them
  private UsbCamera frontcamera;

  private UsbCamera setupServer(int id) {

    UsbCamera camera = CameraServer.startAutomaticCapture("Video Feed", id);
    ((MjpegServer) CameraServer.getServer()).setCompression(50);
    return camera;
  }

  public void useFrontCamera() {
    frontcamera = setupServer(0);
    frontcamera.setResolution(320, 240);
    frontcamera.setFPS(30);

    CameraServer.getServer().setSource(frontcamera);
    System.out.println("Switching to front camera");
    SmartDashboard.putString("Current Camera", "Front");
  }
}
