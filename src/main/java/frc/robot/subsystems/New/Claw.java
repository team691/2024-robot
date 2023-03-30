// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.New;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Claw extends SubsystemBase {
  private final CANSparkMax intake = new CANSparkMax(ArmConstants.intakeMotorID,  MotorType.kBrushless); // mini neo gripper motor
  private final WPI_TalonFX wristMotor = new WPI_TalonFX(ArmConstants.wristMotorID);

  /** Creates a new Claw. */
  public Claw() {
    
    intake.setIdleMode(IdleMode.kBrake);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSupplyCurrentLimit(null, 500);
    intake.setSmartCurrentLimit(30, 28);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void telopClawControls(double up, double down) {
    if (up > down){
      wristMotor.set(up/3);
    }
    else if (down > up){
      wristMotor.set(-down/3);
    }
    else{
      wristMotor.stopMotor();
    }
  }

  public CommandBase feedIntake() {
    return run(
         () -> {
    intake.set(ArmConstants.defaultGripperSpeed);
    });
  }

  public CommandBase disposeIntake() {
    return run(
      () -> {
    intake.set(-ArmConstants.defaultGripperSpeed);
    });
  }

  public CommandBase stillIntake() {
    return run(
         () -> {
    intake.stopMotor();;
    });
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
