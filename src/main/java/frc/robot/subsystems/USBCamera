<<<<<<< HEAD
/**import edu.wpi.first.cameraserver;
import edu.wpi.first.vision;
import edu.wpi.first.cscore; 
import edu.wpi.first.TimedRobot;
=======
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
>>>>>>> 7c9d0229ba3d7abd98c352a20cdf7eecedfcb929
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class USBCamera extends SubsystemBase {
    UsbCamera usbCamera = new UsbCamera("Usb Camera", 0);
    
    // private final int quality = 50;
    // private final String name = "cam0";
    public USBCamera() {	
	    usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(640,480);
        SmartDashboard.putData((Sendable) usbCamera);
    }
<<<<<<< HEAD

}**/
=======
}
>>>>>>> 7c9d0229ba3d7abd98c352a20cdf7eecedfcb929
