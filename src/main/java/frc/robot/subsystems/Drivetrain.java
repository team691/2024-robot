package frc.robot.subsystems;

// BASIC MOTORS AND ENCODERS
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// GYRO
// import java.time.*;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
//import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
//import edu.wpi.first.wpilibj.SPI.Port;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
// DIFFERENTIAL DRIVE
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
// COMMANDS
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// CONSTANTS
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;



public class DriveTrain extends SubsystemBase {

//Motor control group left
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(DriveConstants.kFrontLeftMotorID, MotorType.kBrushless);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(DriveConstants.kRearLeftMotorID, MotorType.kBrushless);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeftMotor, m_rearLeftMotor);

//Motor control group right
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(DriveConstants.kFrontRightMotorID, MotorType.kBrushless);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(DriveConstants.kRearRightMotorID, MotorType.kBrushless);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontRightMotor, m_rearRightMotor);

  //Motors defined in Arm subsystem, possibly uncomment right here
  /*private final CANSparkMax m_gripperMotor = new CANSparkMax(DriveConstants.kGripperMotorID, MotorType.kBrushless);
   *private final CANSparkMax m_verticalMotor = new CANSparkMax(DriveConstants.kVerticalMotorID, MotorType.kBrushless);
   *private final CANSparkMax m_telescopingMotor = new CANSparkMax(DriveConstants.kTelescopingMotorID, MotorType.kBrushless);
   */

   // The robot's drive
   //private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
   DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

   /* encoders but more condensed? */

   // Left-side drive encoder
   private final Encoder m_leftEncoder =
   new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed
   );

   // Right-side drive encodder
   private final Encoder m_rightEncoder =
   new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed
   );
  
   //private final ADIS16470_IMU gyro = new ADIS16470_IMU();
   //private final ADIS16470_IMU gyro = new ADIS16470_IMU(IMUAxis.kY, Port.kMXP, CalibrationTime._32ms);
   private final AHRS navx = new AHRS(SerialPort.Port.kUSB);

   double angle;

   public DriveTrain() {
      m_frontLeftMotor.setSmartCurrentLimit(40, 40);
      m_frontRightMotor.setSmartCurrentLimit(40, 40);
      m_rearLeftMotor.setSmartCurrentLimit(40, 40);
      m_rearRightMotor.setSmartCurrentLimit(40, 40);
      // Sets the distance per pulse for the encoders
      m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      // We need to invert one side of the drivetrain so that positive voltages
      // result in both sides moving forward. 
      m_right.setInverted(true);
      navx.reset();
      navx.calibrate();
     // gyro.setYawAxis(IMUAxis.kX);
   }
   
   /* DRIVE IG */
   
   
   // Controller-based drive
   public void drive (double xSpeed, double zRotation) {
      m_drive.arcadeDrive(xSpeed, zRotation);
   }

   /* option 2?
   public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
      return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
          .withName("arcadeDrive");
    } */
   // Autonomous driving commands

   // referenced from RapidReactCommandBot Example as supplied by WPILIB

  /* public CommandBase driveDistanceCommand(double distanceMeters, double speed, double zRotation) {
      return runOnce(
         () -> {
            // Reset encoders at the start of the command
            m_leftEncoder.reset();
            m_rightEncoder.reset();
         })
      // Drive forward at specified speed
      .andThen(run(() -> m_drive.arcadeDrive(speed, zRotation)))
      // End command when we've traveled the specified distance
      .until (
         () ->
         Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance())
         >= distanceMeters)
      // Stop the drive when the command ends
      .finallyDo(interrupted -> m_drive.stopMotor());
   }*/ 

   /* SLOWER MODE OPTION THING */
   public void setMaxOutput(double maxOutput) {
      m_drive.setMaxOutput(maxOutput);
   }

   /* ENCODER COMMANDS */
   
   // Resets the drive encoders to currently read a position of 0.
   public void resetEncoders() {
      m_leftEncoder.reset();
      m_rightEncoder.reset();
   }

   //Gets the left drive encoder.
   public Encoder getLeftEncoder() {
      return m_leftEncoder;
   }

   // Gets the  right drive encoder.
   public Encoder getRightEncoder() {
      return m_rightEncoder;
   }

   /* GYRO STUFF */

   public double getAngle() {
      double angle = navx.getAngle();
      System.out.println(angle);
      return angle;
   }

   // Code to check angle, run until the angle is zero, then set motors to zero when
   public CommandBase balanceCommand () {
      return run(
         () -> {
            getAngle();
            // Reset encoders at the start of the command
            if (getAngle() >= .5) {
            SmartDashboard.putString("Check", "Upward");
            m_drive.arcadeDrive(AutoConstants.kBalancingSpeed, 0);
            } else {
               if (getAngle() <= -.5) {
                  SmartDashboard.putString("Check", "Downward");
                  m_drive.arcadeDrive(-AutoConstants.kBalancingSpeed, 0);
               }
            }
         })
      // End command when close enough to 0 angle
      .until (
         () ->
            getAngle() <= (.5) && getAngle() >= (-.5))
      // Stop the drive when the command ends
      .finallyDo(interrupted -> m_drive.stopMotor());
   }

   public double getAverageEncoderDistance() {
      return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
   }
}
