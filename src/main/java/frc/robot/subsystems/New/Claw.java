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
  private final CANSparkMax intake = new CANSparkMax(ArmConstants.intakeMotorID,  MotorType.kBrushless); // mini neo gripper motor
  private final CANSparkMax wristMotor = new CANSparkMax(ArmConstants.wristMotorID, MotorType.kBrushless); // telescoping motor

  /** Creates a new Claw. */
  public Claw() {}

  @Override
  public void periodic() {
    intake.setIdleMode(IdleMode.kBrake);
    wristMotor.setIdleMode(IdleMode.kBrake);

    // This method will be called once per scheduler run
  }

  public void ClawControls(double open, double close, double rotation) {
    wristMotor.set(rotation/18); // change division number to adjust speed for more precision

    if (open > close){
      intake.set(open/6);
    }
    else if (close > open){
      intake.set(-close/6);
    }
    else{
      intake.stopMotor();
    }
  }

  public void openGripper() {
    intake.set(ArmConstants.defaultGripperSpeed);
  }

  public void closeGripper() {
    intake.set(-ArmConstants.defaultGripperSpeed*1.75);
  }

  public void stillGripper() {
    intake.stopMotor();
  }

  public void upClaw() {
    wristMotor.set(ArmConstants.defaultRotationSpeed);
  }

  public void downClaw() {
    wristMotor.set(-ArmConstants.defaultRotationSpeed);
  }

  public void stillClaw() {
    wristMotor.stopMotor();
  }
}
