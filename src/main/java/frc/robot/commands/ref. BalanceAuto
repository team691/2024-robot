package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.New.Arm;

public class BalanceAuto extends SequentialCommandGroup {

 /*  public Command BalanceAutonomous(DriveTrain m_drive){
    return Autos.goForward(m_drive)
    .andThen(Autos.goBackward(m_drive))
    .andThen(Autos.balanceEnergyStation(m_drive));
  }*/

  
  public BalanceAuto(DriveTrain drive) {
    addCommands(
        // Drive forward the specified distance
        new DriveDistanceCommand(
            AutoConstants.kAutoDriveDistanceInchesBalance, AutoConstants.kAutoDriveSpeed,0, drive),
        new BalanceCommand(drive)
    );
  }
   
}
  
