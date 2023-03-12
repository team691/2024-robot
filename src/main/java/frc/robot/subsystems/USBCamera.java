package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.cscore.CvSink;
// import edu.wpi.cscore.CvSource;
// import edu.wpi.first.TimedRobot;
// import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class USBCamera extends SubsystemBase {
    UsbCamera usbCamera = new UsbCamera("Usb Camera", 0);
    
    // private final int quality = 50;
    // private final String name = "cam0";
    public USBCamera() {	
	    usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(1088,816);
        SmartDashboard.putData((Sendable) usbCamera);
    }

}
