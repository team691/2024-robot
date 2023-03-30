// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.New;

//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intake = new CANSparkMax(ArmConstants.intakeMotorID,  MotorType.kBrushless); // mini neo gripper motor
  //private final WPI_TalonFX wristMotor = new WPI_TalonFX(ArmConstants.wristMotorID);

  /** Creates a new Claw. */
  public Intake() {
    
    intake.setIdleMode(IdleMode.kBrake);
    //wristMotor.setNeutralMode(NeutralMode.Brake);
    //wristMotor.configSupplyCurrentLimit(null, 500);
    intake.setSmartCurrentLimit(30, 28);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
