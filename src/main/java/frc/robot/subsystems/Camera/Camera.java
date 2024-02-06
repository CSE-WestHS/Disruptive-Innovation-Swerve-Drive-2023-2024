// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Camera extends SubsystemBase {
  // Creates UsbCamera and MjpegServer [1] and connects them
  UsbCamera CamerServer =
      CameraServer.startAutomaticCapture("USB Camera 1", "C:\\Users\\programmer\\Videos");

  // Creates the CvSink and connects it to the UsbCamera
  CvSink cvSink = CameraServer.getVideo();

  // Creates the CvSource and MjpegServer [2] and connects them
  CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
}
