// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.armControls.ControlExtensionCommand;
import frc.robot.commands.armControls.ControlGripperCommand;
import frc.robot.commands.armControls.HighGoalCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.New.Arm;
//import frc.robot.subsystems.New.ArmPosition;
import frc.robot.subsystems.New.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAuto extends SequentialCommandGroup {
  /** Creates a new ScoreAuto. */
  public ScoreAuto(DriveTrain drive, Arm arm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // did we ever configure this to work with an intake rather than the gripper?
    addCommands(
      new HighGoalCommand(arm),
      new ControlExtensionCommand(arm, claw, true),
      new ControlGripperCommand(claw, false),
      //new DriveTimeCommand(-AutoConstants.kAutoDriveSpeed,0, drive, .03),
      new DriveTimeCommand(AutoConstants.kAutoDriveSpeed, 0 , drive, 4.4)
      //, new LimelightTrackingCommand(lime, drive)
    );
  }
}
