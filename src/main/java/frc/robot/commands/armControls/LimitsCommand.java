// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armControls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.ArmPosition;

public class LimitsCommand extends InstantCommand {
  private final Arm m_arm;
  private final ArmPosition m_position;
  /** Creates a new LimitsCommand. */
  public LimitsCommand(Arm arm, ArmPosition position) {
    m_arm = arm;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    m_arm.stopRotate();
    m_arm.setArmPosition(m_position);
  }
}
