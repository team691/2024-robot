/**import edu.wpi.first.cameraserver;
import edu.wpi.first.vision;
import edu.wpi.first.cscore; 
import edu.wpi.first.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.CameraServer;

public class USBCamera extends SubsystemBase {

    CameraServer server;
    private final int quality = 50;
    private final String name = "cam0";

    public Camera() {	
	    server = CameraServer.getInstance();
        server.setQuality(quality);
        server.startAutomaticCapture(name);
    }

}**/