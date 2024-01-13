// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveOutOfCommunityAuto;
import frc.robot.commands.testCommands.RedAlliance;
// subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.LimelightTargetTrack;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
// subsystems, commands, and trigger mappings) should be declared here.

public class RobotContainer {
  // exampleeee
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drive = new DriveTrain();
  private final Limelight m_lime = new Limelight();

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //XboxController opControls = new XboxController(OperatorConstants.kXboxControllerPort);
  Joystick stick = new Joystick(OperatorConstants.kStick1ControllerPort);
  Joystick stick2 = new Joystick(OperatorConstants.kStick2ControllerPort);

  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);

  Trigger topTrigger = new Trigger(topLimitSwitch::get);
  Trigger bottomTrigger = new Trigger(bottomLimitSwitch::get);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Configure default commands

    // Set the default drive command to split-stick arcade drive
    m_drive.setDefaultCommand(
            // Makes robot drive
          new RunCommand(
              () ->
                  m_drive.drive(
                  -stick.getY()/1.1,
                  -stick2.getZ()/1.1
                        ), m_drive
                ) 
                );

    // Add commands to the autonomous command chooser
    /*m_chooser.setDefaultOption("Simple Auto", );
    m_chooser.addOption("Complex Auto", m_complexAuto); */
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    m_chooser.addOption("Drive out of community", new DriveOutOfCommunityAuto(m_drive));

    // m_chooser.addOption("Default Auto", new RedAlliance());
    // m_chooser.addOption("My Auto", kCustomAuto); --> need to fix later
    SmartDashboard.putData("Auto Choices", m_chooser);
    SmartDashboard.putData(m_chooser);
  }


  
  private void configureBindings() {

    // Schedules the motor output to be half max output when the right bumper is pressed
    new JoystickButton(stick, 1)
    .whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(() -> m_drive.setMaxOutput(1))); 
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // debounces exampleButton with a 0.1s debounce time, rising edges only
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return m_chooser.getSelected();
  }
}
