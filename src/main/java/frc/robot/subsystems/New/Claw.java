// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.New;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
  private final CANSparkMax gripperMotor = new CANSparkMax(ArmConstants.kGripperMotorID,  MotorType.kBrushless); // mini neo gripper motor
  private final CANSparkMax rotationMotor = new CANSparkMax(ArmConstants.rotationMotorID, MotorType.kBrushless); // telescoping motor

  /** Creates a new Claw. */
  public Claw() {}

  @Override
  public void periodic() {
    gripperMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    // This method will be called once per scheduler run
  }

  public void ClawControls(double open, double close, double rotation) {
    rotationMotor.set(rotation/4);
    if (open > close){
      gripperMotor.set(open/6);
    }
    else if (close > open){
      gripperMotor.set(-close/6);
    }
    else{
      gripperMotor.stopMotor();
    }

  }

  public void openGripper() {
    gripperMotor.set(ArmConstants.defaultGripperSpeed);
  }

  public void closeGripper() {
    gripperMotor.set(-ArmConstants.defaultGripperSpeed*1.75);
  }

  public void stillGripper() {
    gripperMotor.stopMotor();
  }
}
