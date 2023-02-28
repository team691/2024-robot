// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.ArmPosition;

public class raiseCommand extends CommandBase {
  private final Arm m_arm;
  private final ArmPosition m_position;
  /** Creates a new raiseCommand. */
  public raiseCommand(Arm arm, ArmPosition position) {
    m_arm = arm;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (m_position){

      case GROUND:
        Autos.returnToFloorAuto(m_arm);

      case LOW:
        Autos.lowGoalAuto(m_arm);

      case MEDIUM:
        Autos.midGoalAuto(m_arm);

      case HIGH:
        Autos.highGoalAuto(m_arm);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
