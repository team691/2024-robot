 package frc.robot.subsystems.New;

 // TODO: Review page: https://docs.wpilib.org/en/2020/docs/software/old-commandbased/commands/limit-switches-control-behavior.html

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Counter;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;*/

//Possible Solution: Comment Out Arm Constants
import frc.robot.Constants.ArmConstants;

import java.time.Duration;
import java.time.Instant;


// Timer instead of encoders for rotation
//import edu.wpi.first.wpilibj.Timer;

public class Arm extends SubsystemBase {
  // Mass measured in grams
  public static final double armMass = 0.0;

  // Lengths measured in <some unit>
  // heads up, constants like min and max lengths and motor channels can be moved
  // over to the constants class for organization purposes
  // TODO: Get maximum/minimum arm lengths
  public static final double maxArmLength = 44.0;
  public static final double minArmLength = 6.0;

  public static final double initialArmLength = 6.0;
  public static final double initialArmAngle = 0.0;

  public static final double initialArmStaticGain = 0.0;
  public static final double initialArmGravityGain = 0.0;
  public static final double initialArmAccelerationGain = 0.0;
  public static final double initialArmVelocityGain = 0.0;
  public static final double initialArmVelocity = 0.0;

  // Feedforward control for arm
  private ArmFeedforward feedforward;

  // Motors
  private final WPI_VictorSPX extensionMotor = new WPI_VictorSPX(ArmConstants.extensionMotorID); // chain motor
  private final WPI_TalonFX rotationMotor = new WPI_TalonFX(ArmConstants.rotationMotorID); // telescoping motor
//  private final CANSparkMax gripperMotor = new CANSparkMax(ArmConstants.kGripperMotorID,  MotorType.kBrushless); // mini neo gripper motor

  // Motor encoders

  // ARM POSITION AUTOMATED
  // Timers
 /*private final Timer rotationTime = new Timer();
  private final Timer gripperTime = new Timer();
  private final Timer feedForwardTime = new Timer();
  */

  public ArmPosition armPosition = ArmPosition.LOW;

  // TODO: Setup motor encoders

  private double lastArmLength;
  private double lastArmAngle;

  private double currentArmLength;
  private double currentArmAngle;

  private double lastArmVelocity;
  private double currentArmVelocity;

  private Instant lastArmChangeTimestamp;

  // TODO: Edit digital input channel
  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);
  //Counter counter = new Counter(limitSwitch);

  /**
   * Initializes a new Arm object
   */
  public Arm() {
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    this.feedforward = new ArmFeedforward(Arm.initialArmStaticGain, Arm.initialArmGravityGain,
        Arm.initialArmVelocityGain, Arm.initialArmAccelerationGain);

    this.lastArmLength = Arm.initialArmLength;
    this.lastArmAngle = Arm.initialArmAngle;

    this.currentArmAngle = Arm.initialArmLength;
    this.currentArmLength = Arm.initialArmAngle;

    this.lastArmVelocity = Arm.initialArmVelocity;
    this.currentArmVelocity = Arm.initialArmVelocity;

    this.lastArmChangeTimestamp = Instant.now();


  }

  /**
   * Accesses the robot's current arm angle
   * 
   * @return Robot's current arm angle
   */
  public double getCurrentArmAngle() {
    return this.currentArmAngle;
  }

  /**
   * Accesses robot's current arm length
   * 
   * @return Robot's current arm length
   */
  public double getCurrentArmLength() {
    return this.currentArmLength;
  }

  /**
   * Accesses robot's last arm angle
   * 
   * @return Robot's last arm angle
   */
  public double getLastArmAngle() {
    return this.lastArmAngle;
  }

  /**
   * Accesses robot's last arm length
   * 
   * @return Robot's last arm length
   */
  public double getLastArmLength() {
    return this.lastArmLength;
  }

  /**
   * Sets robot's arm angle
   * 
   * @param newAngle Angle to set arm to
   */
  public void setArmAngle(double newAngle) {
    this.lastArmAngle = this.currentArmAngle;
    this.currentArmAngle = newAngle;

    updateFeedforward();
    updateArmHardware();
  }

  /**
   * Sets robot's arm length
   * 
   * @param newLength Length to set arm to
   */
  public void setArmLength(double newLength) {
    this.lastArmLength = this.currentArmLength;
    this.currentArmLength = newLength;

    updateFeedforward();
    updateArmHardware();
  }

  public ArmPosition getArmPosition() {
    return armPosition;
  }

  public void setArmPosition(ArmPosition position) {
    armPosition = position;
  }

  /**
   * Calculates the static gain of the robot arm from one moment to another
   * 
   * @param startTime The start moment
   * @param endTime   The end moment
   * @return Static gain
   */
  private double calculateStaticGain(Instant startTime, Instant endTime) {
    //Finding the change in time
    final double deltaTime = (double) Duration.between(startTime, endTime).toMillis();
    //Coulombs = FRC battery ampere hours * deltaTime
    final double C = 17.6 * (deltaTime);
    //Calculating Force in Newtons to overcome static friction (Mass of Arm * Arm acceleration)
    double NkS = Arm.armMass * calculateAccelerationGain(startTime, endTime);
    //Convert Newtons to Volts (Static Gain in Newtons * Length of Arm / Coulombs)
    double VkS = (NkS * getLastArmLength()) / C;

    return VkS;
  }

  /**
   * Calculates the gravitational gain of the robot arm from one moment to another
   * 
   * @param startTime The start moment
   * @param endTime   The end moment
   * @return Gravitational gain
   */
  private double calculateGravityGain(Instant startTime, Instant endTime) {
    // Dividing by 1 million to convert m/s^2 to m/ms^2
    final double gravitationalAccelerationMillis = 9.8 / 1_000_000;
    final double gravitationalForce = Arm.armMass * gravitationalAccelerationMillis;
    final double deltaTime = (double) Duration.between(startTime, endTime).toMillis();
    final double gravity = gravitationalForce / deltaTime;
    return gravity;
  }

  /**
   * Calculates the velocity gain of the robot arm between two moments
   * 
   * @param startTime The start moment
   * @param endTime   The end moment
   * @return Velocity gain
   */
  private double calculateVelocityGain(Instant startTime, Instant endTime) {
    final double displacement = Math.abs(currentArmLength - lastArmLength);
    final double deltaTime = (double) Duration.between(startTime, endTime).toMillis();
    final double velocity = displacement / deltaTime;

    lastArmVelocity = currentArmVelocity;
    currentArmVelocity = velocity;
    return velocity;
  }

  /**
   * Calculates the acceleration gain of the robot arm between two moments
   * 
   * @param startTime The start moment
   * @param endTime   The end moment
   * @return Acceleration gain
   */
  private double calculateAccelerationGain(Instant startTime, Instant endTime) {
    final double deltaVelocity = Math.abs(currentArmVelocity - lastArmVelocity);
    final double deltaTime = (double) Duration.between(startTime, endTime).toMillis();
    final double acceleration = deltaVelocity / deltaTime;

    return acceleration;
  }

  /**
   * Updates the feedforward control
   */
  private void updateFeedforward() {
    final Instant currentArmChangeTimestampt = Instant.now();

    final double staticGain = calculateStaticGain(lastArmChangeTimestamp, currentArmChangeTimestampt);
    final double gravityGain = calculateGravityGain(lastArmChangeTimestamp, currentArmChangeTimestampt);
    final double velocityGain = calculateVelocityGain(lastArmChangeTimestamp, currentArmChangeTimestampt);
    final double accelerationGain = calculateAccelerationGain(lastArmChangeTimestamp, currentArmChangeTimestampt);

    this.feedforward = new ArmFeedforward(staticGain, gravityGain, velocityGain, accelerationGain);
  }

  /**
   * Updates the robot arm hardware to match the code state
   */
  private void updateArmHardware() {
    final Instant currentArmChangeTimestampt = Instant.now();
    extensionMotor.setVoltage(feedforward.calculate(getCurrentArmAngle(),
        calculateVelocityGain(lastArmChangeTimestamp, currentArmChangeTimestampt),
        calculateAccelerationGain(lastArmChangeTimestamp, currentArmChangeTimestampt)));
    rotationMotor.setVoltage(feedforward.calculate(getCurrentArmAngle(),
        calculateVelocityGain(lastArmChangeTimestamp, currentArmChangeTimestampt),
        calculateAccelerationGain(lastArmChangeTimestamp, currentArmChangeTimestampt)));
  }

  public void teleopArmControls(double extension, double rotation/* , double open, double close*/) /*double verticalSpeed, double telescopingSpeed
                                                                              * , boolean opengripper, boolean
                                                                              * closegripper, boolean stillgripper1,
                                                                              * boolean stillgripper2
                                                                              */ {
    //rotationMotor.set(rotation);
    extensionMotor.set(extension/2);
   /*  if (open > close){
      gripperMotor.set(open/6);
    }
    else if (close > open){
      gripperMotor.set(-close/6);
    }
    else{
      gripperMotor.stopMotor();
    }*/

    rotationMotor.set(rotation/4);
  }

  public void upArm() {
      rotationMotor.set(ArmConstants.defaultRotationSpeed);
  }

  public void downArm() {
      rotationMotor.set(-ArmConstants.defaultRotationSpeed);
  }

  /*public void openGripper() {
    gripperMotor.set(ArmConstants.defaultGripperSpeed);
  }

  public void closeGripper() {
    gripperMotor.set(-ArmConstants.defaultGripperSpeed*1.75);
  }

  public void stillGripper() {
    gripperMotor.stopMotor();
  }
*/
 /*  public CommandBase retractArm(double retractionTime){
    return runOnce(
        () -> {
          while (feedForwardTime.get() < retractionTime){
            extensionMotor.set(-ArmConstants.defaultRotationSpeed);
          }
          extensionMotor.stopMotor();
        });
  }
  public CommandBase extendArm(double extensionTime){
    return runOnce(
        () -> {
          while (feedForwardTime.get() < extensionTime){
            extensionMotor.set(ArmConstants.defaultRotationSpeed);
          }
          extensionMotor.stopMotor();
        });
  }
*/

  public void stopRotate() {
    rotationMotor.stopMotor();
  }

  // PUT IN ARM CLASS
  public void setArmMotorSpeed(double speed) {
    if (speed > 0) {
      if (topLimitSwitch.get()) {
        // Arm is nearby limit
        // TODO: Set arm motor
        rotationMotor.set(0);
      } else {
        rotationMotor.set(speed);
      }
    } else {
      if (bottomLimitSwitch.get()) {
        rotationMotor.set(0);
      } else {
        rotationMotor.set(speed);
      }
    }
  }
}
