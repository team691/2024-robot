// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.New.Arm;

public class ExtentionControlCommand extends CommandBase {
  /** Creates a new ExtentionControlCommand. */
  private final String m_command;
  private final Arm m_arm;
  public ExtentionControlCommand(String extensionCommand, Arm arm) {
    m_command = extensionCommand;
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // not actual code fill in for now
    if (m_command == "extend"){
      Autos.openGripper(m_arm);
        }
    // May want to extend the code to close for specifically game peices
      else if (m_command == "retract"){
       Autos.closeGripper(m_arm);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
