// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// ooh funky little command imports 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/* import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;*/
//import edu.wpi.first.wpilibj2.command.button.Trigger;
// constants
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BalanceAuto;
import frc.robot.commands.BalanceCommand;
//import frc.robot.commands.BalanceAuto;
import frc.robot.commands.DriveOutOfCommunityAuto;
import frc.robot.commands.LimelightTrackingCommand;
import frc.robot.commands.ScoreAuto;
//import frc.robot.commands.armControls.ControlGripperCommand;
//import frc.robot.commands.ScoreAuto;
// import frc.robot.commands.gripperControlCommand;
import frc.robot.subsystems.New.Arm;
//import frc.robot.subsystems.New.ArmPosition;
import frc.robot.subsystems.New.Claw;

// stuff for the examples because templete
//import static edu.wpi.first.wpilibj2.command.Commands.parallel;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;

// subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.LimelightTargetTrack;

/* 
// camera
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;*/

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
// subsystems, commands, and trigger mappings) should be declared here.

public class RobotContainer {
  // exampleeee
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drive = new DriveTrain();
  private final Arm m_arm = new Arm();
  private final Claw m_claw = new Claw();
  private final Limelight m_lime = new Limelight();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController opControls = new XboxController(OperatorConstants.kXboxControllerPort);
  /* Trigger yButton = new JoystickButton(opControls, XboxController.Button.kY.value);
  Trigger xButton = new JoystickButton(opControls, XboxController.Button.kX.value);
  Trigger aButton = new JoystickButton(opControls, XboxController.Button.kA.value);
  Trigger bButton = new JoystickButton(opControls, XboxController.Button.kB.value);
  Trigger leftBumper = new JoystickButton(opControls, XboxController.Button.kLeftBumper.value);
  Trigger rightBumper = new JoystickButton(opControls, XboxController.Button.kRightBumper.value); */
  
  Joystick stick = new Joystick(OperatorConstants.kStick1ControllerPort);
  Joystick stick2 = new Joystick(OperatorConstants.kStick2ControllerPort);

  
  // TODO: Edit digital input channel
  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);

  Trigger topTrigger = new Trigger(topLimitSwitch::get);
  Trigger bottomTrigger = new Trigger(bottomLimitSwitch::get);


  
  /*CommandXboxController xboxControl = new CommandXboxController(OperatorConstants.kXboxControllerPort); //extend arm
  CommandXboxController rightstick = new CommandXboxController(OperatorConstants.kXboxControllerPort); //vertical arm*/
 // XboxController buttons = new XboxController(OperatorConstants.kXboxControllerPort); //open and close gripper
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  //private final Command BalanceAuto = new BalanceAuto(m_drive);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
   // displayCamera();

   /*  m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -stick.getY(), () -> (stick2.getZ())/1.2));*/ 
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

    m_arm.setDefaultCommand(
      new RunCommand(
        () ->
            m_arm.teleopArmControls(
              opControls.getLeftY(), // telescoping
              opControls.getRightY()//, // rotation
              /*opControls.getLeftTriggerAxis(), // open
              opControls.getRightTriggerAxis() // close
            /* 
            buttons.getRightBumperPressed(), //open gripper
            buttons.getLeftBumperPressed(), //close gripper
            buttons.getLeftBumperReleased(), //stop closing
            buttons.getRightBumperReleased() //stop opening*/
            ), m_arm
            //sticks on the "xbox" controller for extension and right stick for up and down)
            //third argument sets the gripper itself to open and close
            )
          );

    m_claw.setDefaultCommand(
      new RunCommand(
        () ->
        m_claw.telopClawControls(
          opControls.getLeftTriggerAxis(), // wrist up
          opControls.getRightTriggerAxis() // wrist down
        ), m_claw
        )
    );
        // Add commands to the autonomous command chooser
    /*m_chooser.setDefaultOption("Simple Auto", );
    m_chooser.addOption("Complex Auto", m_complexAuto); */
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    m_chooser.addOption("Balance", new BalanceAuto(m_drive));
    m_chooser.addOption("Score", new ScoreAuto(m_drive, m_arm, m_claw));
    m_chooser.addOption("Drive out of community", new DriveOutOfCommunityAuto(m_drive, m_lime));
    SmartDashboard.putData(m_chooser);

    // Put the chooser on the dashboard
    //Shuffleboard.getTab("Autonomous").add(m_autoChooser);
   // initializeAutoChooser();
  }


  // Use this method to define your trigger->command mappings. Triggers can be created via the
  // {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
  // predicate, or via the named factories in {@link
  // edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
  // CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
  // PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
  // joysticks}.
  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   /* new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem)); */

    // Schedules the motor output to be half max output when the right bumper is pressed
    // idk man its a button
    new JoystickButton(stick, 1)
    .whileTrue(new InstantCommand(() -> m_drive.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(() -> m_drive.setMaxOutput(1))); 
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // debounces exampleButton with a 0.1s debounce time, rising edges only
    new Trigger(topLimitSwitch::get)
    .onTrue(m_arm.stopRotate());

    new Trigger(bottomLimitSwitch::get)
    .onTrue(m_arm.stopRotate());
  /*  new JoystickButton(opControls, XboxController.Button.kX.value)
      .onTrue(m_arm.returnToFloor());

    new JoystickButton(opControls, XboxController.Button.kA.value)
      .onTrue(m_arm.lowGoal());

    new JoystickButton(opControls, XboxController.Button.kB.value)
      .onTrue(m_arm.midGoal());

    new JoystickButton(opControls, XboxController.Button.kY.value)
      .onTrue(m_arm.highGoal().withTimeout(2)); */

    /*new JoystickButton(opControls, XboxController.Button.kLeftBumper.value)
      .onTrue(new ControlGripperCommand(m_claw, false));

    new JoystickButton(opControls, XboxController.Button.kRightBumper.value)
      .onTrue(new ControlGripperCommand(m_claw, true)); */
    
    /* TEST */
    new JoystickButton(opControls, XboxController.Button.kStart.value)
      .onTrue(new BalanceCommand(m_drive));
    
    new JoystickButton(opControls, XboxController.Button.kBack.value)
      .onTrue(m_drive.getAngleTest());

    new JoystickButton(opControls, XboxController.Button.kLeftBumper.value)
      .onTrue(m_claw.feedIntake())
      .onFalse(m_claw.stillIntake());

      new JoystickButton(opControls, XboxController.Button.kLeftBumper.value)
      .onTrue(m_claw.disposeIntake())
      .onFalse(m_claw.stillIntake());


    //Testing LimelightTracking

    new JoystickButton(opControls, XboxController.Button.kA.value)
      .onTrue(new LimelightTrackingCommand(m_lime, m_drive));

    /* 
    //Testing Encoders

    new JoystickButton(stick, 7)
      .onTrue(m_drive.printRearRightEncoderDistance());
    
    new JoystickButton(stick, 8)
      .onTrue(m_drive.printFrontRightEncoderDistance());
    
    new JoystickButton(stick, 9)
      .onTrue(m_drive.printRearLeftEncoderDistance());
    
    new JoystickButton(stick, 10)
      .onTrue(m_drive.printFrontLeftEncoderDistance());*/
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
