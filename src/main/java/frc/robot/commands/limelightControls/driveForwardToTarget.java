// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelightControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class driveForwardToTarget extends CommandBase {
  private final Timer m_timer = new Timer();
  private final double m_timeout;
  private final Limelight m_lime;
  private final DriveTrain m_drive;
  /** Creates a new driveForwardToTarget. */
  public driveForwardToTarget(Limelight lime, DriveTrain drive, double timeout) {
    m_timeout = timeout;
    m_lime = lime;
    m_drive = drive;
    addRequirements(m_drive, m_lime);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    //m_drive.drive(m_speed, m_rotation);
    System.out.println("DRIVE DISTANCE");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(AutoConstants.kAutoDriveSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lime.m_LimelightTargetArea == 50;
    }
}

