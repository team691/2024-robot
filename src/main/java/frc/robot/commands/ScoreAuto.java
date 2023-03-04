// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.New.Arm;
import frc.robot.subsystems.New.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAuto extends SequentialCommandGroup {
  /** Creates a new ScoreAuto. */
  public ScoreAuto(DriveTrain drive, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new raiseCommand(arm, ArmPosition.BAR),
      new raiseCommand(arm, ArmPosition.LOW),
      new gripperControlCommand("open", arm),
     // new DriveDistanceCommand(AutoConstants.kAutoDriveDistanceInchesF, AutoConstants.kAutoDriveSpeed,0, drive),
      new DriveDistanceCommand(AutoConstants.kAutoDriveDistanceInchesB, AutoConstants.kAutoDriveSpeed,0 , drive)
    );
  }
}
