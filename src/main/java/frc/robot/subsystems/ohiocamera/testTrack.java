// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ohiocamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;



/** Add your docs here. */
public class testTrack {
   UsbCamera usbCamera = new UsbCamera("USB Camera 0",0); 
   MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
   
}
