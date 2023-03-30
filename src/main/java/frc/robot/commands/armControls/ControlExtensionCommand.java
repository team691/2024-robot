// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.Intake;
import frc.robot.subsystems.New.Wrist;

public class ControlExtensionCommand extends CommandBase {
  private final Arm m_arm;
  private final Wrist m_claw;
  private final Timer m_timer = new Timer();
  private double m_timeout = 0;
  private boolean m_extend = false;

  /** Creates a new ControlGripperCommand. */
  public ControlExtensionCommand(Arm arm, Wrist claw, boolean extend) {
    m_arm = arm;
    m_claw = claw;
    m_extend = extend;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_extend){
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
    if (m_extend){
    //  m_claw.intake();
    }
    else{
    //  m_claw.openGripper();
    };
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_claw.stillGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_timeout;
  }
}
