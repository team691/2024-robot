// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.Claw;

public class ControlGripperCommand extends CommandBase {
  private final Claw m_claw;
  private final Timer m_timer = new Timer();
  private double m_timeout = 0;
  private boolean m_close = false;


  /** Creates a new ControlGripperCommand. */
  public ControlGripperCommand(Claw claw, boolean close) {
    m_claw = claw;
    m_close = close;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_close){
    m_timeout = ArmConstants.closeGripperTime;
    }
    else{
    m_timeout = ArmConstants.openGripperTime;
    }
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_close){
      m_claw.closeGripper();
    }
    else{
      m_claw.openGripper();
    };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stillGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_timeout;
  }
}
