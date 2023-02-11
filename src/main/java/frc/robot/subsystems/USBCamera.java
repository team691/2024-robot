package frc.robot.subsystems;

// // import edu.wpi.first.wpilibj.CameraServer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class USBCamera extends SubsystemBase {
//     UsbCamera usbCamera = new UsbCamera("Usb Camera", 1);
    
//     // private final int quality = 50;
//     // private final String name = "cam0";
//     public USBCamera() {	
// 	    usbCamera = CameraServer.startAutomaticCapture();
//         usbCamera.setResolution(320, 240);
//         SmartDashboard.putData((Sendable) usbCamera);
//     }
// }

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
 * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
 * to the dashboard. Just add this to the robotInit() method in your program.
 */
public class USBCamera extends TimedRobot {
  @Override
  
  public void robotInit() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    CameraServer.startAutomaticCapture();
    CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
    outputStream.putFrame(null);

    SmartDashboard.putData((Sendable) outputStream);
  }
}
