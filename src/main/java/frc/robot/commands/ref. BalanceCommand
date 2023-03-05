package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class BalanceCommand extends CommandBase{
  private final DriveTrain m_drive;
  
    public BalanceCommand(DriveTrain drive) {
        m_drive = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
      }
     // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Autos.balanceEnergyStation(m_drive);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }
}
