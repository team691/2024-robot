package frc.robot.commands.armControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.ArmPosition;

public class LowGoalCommand extends CommandBase{
    private final Arm m_arm;
    private final Timer m_timer = new Timer();
    private double m_timeout = 0;
    private boolean upGoal = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public LowGoalCommand(Arm arm) {
      m_arm = arm;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_arm);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_arm.getArmPosition() == ArmPosition.GROUND) {
          m_timeout = ArmConstants.floorToLow;
          upGoal = true;
        } else if (m_arm.getArmPosition() == ArmPosition.MEDIUM) {
          m_timeout = ArmConstants.midToLow;
          upGoal = false;
        } else if (m_arm.getArmPosition() == ArmPosition.HIGH) {
            m_timeout =ArmConstants.highToLow;
            upGoal = false;
        } else if (m_arm.getArmPosition() == ArmPosition.BAR){
            m_timeout = ArmConstants.barToLow;
            upGoal = false;
        }

        m_arm.setArmPosition(ArmPosition.MEDIUM);
                
        m_timer.reset();
        m_timer.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(upGoal)
            m_arm.upArm();
        else
            m_arm.downArm();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.stopRotate();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_timer.get() >= m_timeout;
    }
  
}
