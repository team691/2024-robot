// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
/* import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;*/
//import edu.wpi.first.wpilibj2.command.button.Trigger;
// constants
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BalanceAuto;
import frc.robot.commands.ScoreAuto;
// import frc.robot.commands.gripperControlCommand;
import frc.robot.subsystems.New.Arm;
//import frc.robot.subsystems.New.ArmPosition;

// stuff for the examples because templete
//import static edu.wpi.first.wpilibj2.command.Commands.parallel;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;

// subsystems
import frc.robot.subsystems.DriveTrain;

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
                  -stick.getY(),
                  -stick2.getZ()/1.3
                        ), m_drive
                ) 
                );

    m_arm.setDefaultCommand(
      new RunCommand(
        () ->
            m_arm.teleopArmControls(
              opControls.getLeftY(), // telescoping
              opControls.getRightY(), // rotation
              (opControls.getLeftTriggerAxis()), // open
              (opControls.getRightTriggerAxis()) // close
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

        // Add commands to the autonomous command chooser
    /*m_chooser.setDefaultOption("Simple Auto", );
    m_chooser.addOption("Complex Auto", m_complexAuto); */
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    m_chooser.addOption("Balance", new BalanceAuto(m_drive));
    m_chooser.addOption("Score", new ScoreAuto(m_drive, m_arm));
    SmartDashboard.putData(/*"Auto Selecter",*/ m_chooser);

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

    new JoystickButton(opControls, XboxController.Button.kX.value)
      .onTrue(m_arm.returnToFloor());

    new JoystickButton(opControls, XboxController.Button.kA.value)
      .onTrue(m_arm.lowGoal());

    new JoystickButton(opControls, XboxController.Button.kB.value)
      .onTrue(m_arm.midGoal());

    new JoystickButton(opControls, XboxController.Button.kY.value)
      .onTrue(m_arm.highGoal());

    new JoystickButton(opControls, XboxController.Button.kLeftBumper.value)
      .onTrue(m_arm.openGripper());

    new JoystickButton(opControls, XboxController.Button.kRightBumper.value)
      .onTrue(m_arm.closeGripper());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return m_chooser.getSelected(); /*Autos.goForward(m_drive)
      .andThen(Autos.goBackward(m_drive))
      .andThen(Autos.balanceEnergyStation(m_drive));*/
   /*  return m_drive
        .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
        .withTimeout(AutoConstants.kTimeoutSeconds);*/
      //return Autos.balanceEnergyStation(m_drive);
  }
/*   public void displayCamera() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    CameraServer.startAutomaticCapture();
    CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
    outputStream.putFrame(null);
    SmartDashboard.putData((Sendable) outputStream);
  }*/

  public void initializeAutoChooser(){
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    m_chooser.addOption("Balance", new BalanceAuto(m_drive));
    m_chooser.addOption("Score", new ScoreAuto(m_drive, m_arm));
    SmartDashboard.putData( m_chooser);
  }

}
