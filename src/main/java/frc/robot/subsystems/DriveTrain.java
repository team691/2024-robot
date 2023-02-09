package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
 
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(2, MotorType.kBrushless);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeftMotor, m_rearLeftMotor);


  private final CANSparkMax m_frontRightMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRightMotor, m_rearRightMotor);

   // The robot's drive
   //private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);


  // The front-left-side drive encoder
   private final Encoder m_frontLeftEncoder =
   new Encoder(
       DriveConstants.kFrontLeftEncoderPorts[0],
       DriveConstants.kFrontLeftEncoderPorts[1],
       DriveConstants.kFrontLeftEncoderReversed);
       
// The rear-left-side drive encoder
   private final Encoder m_rearLeftEncoder =
      new Encoder(
       DriveConstants.kRearLeftEncoderPorts[0],
       DriveConstants.kRearLeftEncoderPorts[1],
       DriveConstants.kRearLeftEncoderReversed);

// The front-right--side drive encoder
   private final Encoder m_frontRightEncoder =
      new Encoder(
       DriveConstants.kFrontRightEncoderPorts[0],
       DriveConstants.kFrontRightEncoderPorts[1],
       DriveConstants.kFrontRightEncoderReversed);

   // The rear-right-side drive encoder
   private final Encoder m_rearRightEncoder =
      new Encoder(
       DriveConstants.kRearRightEncoderPorts[0],
       DriveConstants.kRearRightEncoderPorts[1],
       DriveConstants.kRearRightEncoderReversed); 

public DriveTrain() {
      // Sets the distance per pulse for the encoders
      m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      // We need to invert one side of the drivetrain so that positive voltages
      // result in both sides moving forward. 
      m_right.setInverted(true);
      }
   
   /* DRIVE IG */
   public void drive (double xSpeed, double zRotation){
      m_drive.arcadeDrive(xSpeed, zRotation);
   }

   /* SLOWER MODE OPTION THING */
   public void setMaxOutput(double maxOutput) {
      m_drive.setMaxOutput(maxOutput);
   }


   /* ENCODER COMMANDS */
   
   // Resets the drive encoders to currently read a position of 0.
   public void resetEncoders() {
      m_frontLeftEncoder.reset();
      m_rearLeftEncoder.reset();
      m_frontRightEncoder.reset();
      m_rearRightEncoder.reset();
   }
   //Gets the front left drive encoder.
   public Encoder getFrontLeftEncoder() {
      return m_frontLeftEncoder;
   }
   // Gets the rear left drive encoder.
   public Encoder getRearLeftEncoder() {
      return m_rearLeftEncoder;
   }
   // Gets the front right drive encoder.
   public Encoder getFrontRightEncoder() {
      return m_frontRightEncoder;
   }
   // Gets the rear right drive encoder. 
   public Encoder getRearRightEncoder() {
      return m_rearRightEncoder;
   }
  }
