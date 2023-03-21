// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
//import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.Claw;

public class ControlWrist extends CommandBase {
  private final Claw m_claw;
  private final Timer m_timer = new Timer();
  private double m_timeout = 0;
  //private boolean m_close = false;
  private boolean m_down = false;

  public ControlWrist(Claw claw, boolean down) {
    m_claw = claw;
    m_down = down;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_down){
    m_timeout = ArmConstants.lowerGripperTime;
    }
    else{
    m_timeout = ArmConstants.raiseGripperTime;
    }
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_down){
      m_claw.downClaw();
    }
    else{
      m_claw.upClaw();
    };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stillClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_timeout;
  }
}