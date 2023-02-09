// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// stuff fo the examples because templete
//import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

// subsystems
import frc.robot.subsystems.DriveTrain;

// constants
// import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;

// 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// ooh funky little command imports 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 *
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // exampleeee
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drive = new DriveTrain();


  // Replace with CommandPS4Controller or CommandJoystick if needed

  Joystick stick = new Joystick(OperatorConstants.kStick1ControllerPort);
  Joystick stick2 = new Joystick(OperatorConstants.kStick1ControllerPort);
  XboxController c = new XboxController(OperatorConstants.kXboxControllerPort);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
                    -stick.getY(),
                    stick2.getZ()/1.2
                    ),
            m_drive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedules the motor output to be half max output when the right bumper is pressed
    new JoystickButton(c, Button.kRightBumper.value)
    .onTrue(new InstantCommand(() -> m_drive.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(() -> m_drive.setMaxOutput(1)));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return Autos.goForward(m_drive);
   /*  return m_drive
        .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
        .withTimeout(AutoConstants.kTimeoutSeconds);*/
  }
}