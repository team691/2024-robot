// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.New.Arm;
//import frc.robot.subsystems.New.ArmPosition;

public class gripperControlCommand extends CommandBase {
  private final Arm m_arm;
  private final String m_command;
  /** Creates a new ScoreAuto. */
  public gripperControlCommand(String gripperCommand, Arm arm) {
    m_arm = arm;
    m_command = gripperCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /*// Called when the command is initially scheduled.
  @Override
  public void initialize() {} */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_command == "open"){
    Autos.openGripper(m_arm);
      }
  // May want to extend the code to close for specifically game peices
    else if (m_command == "close"){
     Autos.closeGripper(m_arm);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /*// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } */
}
