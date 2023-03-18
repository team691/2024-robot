// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimelightTrackingCommand extends CommandBase {
  private final Limelight m_lime;
  private final DriveTrain m_drive;

  /** Creates a new LimelightTrackingCommand. */
  public LimelightTrackingCommand(Limelight lime, DriveTrain drive) {
    m_lime = lime;
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lime.Update_Limelight_Tracking();

          if (m_lime.hasValidTarget())
          {
                m_drive.arcadeDrive(AutoConstants.kDriveSpeed,m_LimelightSteerCommand);
          }
          else
          {
                m_drive.stopDrive();
          }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  /* Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }*/ 
}
