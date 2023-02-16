package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
    private final CANSparkMax m_gripper = new CANSparkMax(GripperConstants.kGripperMotorID, MotorType.kBrushless);

    public Gripper(){

    }

    public void gripperDrive() {
        
    }
    
}
