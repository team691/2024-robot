package frc.robot;
import java.time.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

    // Creates an ADXRS450_Gyro object on the onboard SPI port
    ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // Initialize function
    @Override
    public void robotInit() {
        gyro.reset();
        gyro.calibrate();
    }

    @Override
    public void autonomousPeriodic() {
        // Get the angle of the gyro
        double angle = gyro.getAngle();
        SmartDashboard.putNumber("Gyro angle", angle);
        // Checking to see if the gyro is level
        
        if (angle >= 2) {
            upBot();
        } else {
            if (angle <= -2)
                downBot();
        }
        /*
        if (angle <= 10 && angle >= -10) {
            SmartDashboard.putString("Check", "Level");
        } else {
            SmartDashboard.putString("Check", "Not Level");
        }
        */
    }

    public void upBot() {
        //Make robot go forward
        m_left.set(0.25);
        m_right.set(0.25); 
    }

    public void downBot() {
        //Make robot go backward
        m_left.set(-0.25);
        m_right.set(-0.25);
    }

}
