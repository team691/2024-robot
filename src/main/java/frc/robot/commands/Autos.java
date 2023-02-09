// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Commands;

// Constants
import frc.robot.Constants.AutoConstants;

// Subsystems
import frc.robot.subsystems.DriveTrain;


public final class Autos {
  // private final DriveTrain m_drive = new DriveTrain();
  /** Example static factory for an autonomous command. */
 /* public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  */

  public static CommandBase goForward(DriveTrain m_drive) {
    return m_drive
    .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed, 0)
    .withTimeout(AutoConstants.kTimeoutSeconds);
  }
  
  public static CommandBase goBackward(DriveTrain m_drive) {
    return m_drive
    .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed, 180)
    .withTimeout(AutoConstants.kTimeoutSeconds);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
