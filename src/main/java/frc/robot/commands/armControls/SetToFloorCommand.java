// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armControls;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.New.Arm;

public class SetToFloorCommand extends CommandBase {
  private final Arm m_arm;
  /** Creates a new SetToFloorCommand. */
  public SetToFloorCommand(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.stopRotate();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
