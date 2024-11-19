 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.camera;

import java.io.OutputStream;

import org.opencv.core.CvException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;

/** 
 * This class sets up the USB Camera and its MjpegServer.
 * More documentation and methods can be added here as needed.
 */
public class testTrack {
    private UsbCamera usbCamera;
    private MjpegServer mjpegServer;

    /**
     * Constructor to initialize the camera and server.
     */
    public testTrack() {
        // Create the USB Camera object, name it "USB Camera 0" and assign it to port 0
        usbCamera = new UsbCamera("USB Camera 0", 0);

        // Start automatic capture for CameraServer
        CameraServer.startAutomaticCapture(usbCamera);

        // Create the MJPEG server and bind it to port 1181
        mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);

        // Linking the server to the camera
        mjpegServer.setSource(usbCamera);
        CvSink cvSink = new CvSink("opencv_USB Camera 0");
        cvSink.setSource(usbCamera);

        CvSource outputCvSource = new CvSource("Blur", PixelFormat.kMJPEG,640,480,30);
        MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
        mjpegServer2.setSource(outputCvSource);  
    }
}
