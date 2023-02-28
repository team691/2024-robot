package frc.robot.subsystems.New;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


//Possible Solution: Comment Out Arm Constants
import frc.robot.Constants.ArmConstants;

import java.time.Duration;
import java.time.Instant;

// Timer instead of encoders for rotation
import edu.wpi.first.wpilibj.Timer;

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
  private final WPI_VictorSPX extensionMotor = new WPI_VictorSPX(ArmConstants.extensionMotorChannel); // chain motor
  private final PWMTalonFX rotationMotor = new PWMTalonFX(ArmConstants.rotationMotorChannel); // telescoping motor
  private final PWMSparkMax gripperMotor = new PWMSparkMax(ArmConstants.gripperMotorChannel); // mini neo gripper motor

  // Motor encoders

  // ARM POSITION AUTOMATED
  // Timer
  private final Timer armTime = new Timer();

  public ArmPosition armPosition = ArmPosition.LOW;

  // TODO: Setup motor encoders

  private double lastArmLength;
  private double lastArmAngle;

  private double currentArmLength;
  private double currentArmAngle;

  private double lastArmVelocity;
  private double currentArmVelocity;

  private Instant lastArmChangeTimestamp;

  /**
   * Initializes a new Arm object
   */
  public Arm() {
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

  /**
   * Calculates the static gain of the robot arm from one moment to another
   * 
   * @param startTime The start moment
   * @param endTime   The end moment
   * @return Static gain
   */
  private double calculateStaticGain(Instant startTime, Instant endTime) {
    final double deltaTime = (double) Duration.between(startTime, endTime).toMillis();
    final double C = 17.6 * (deltaTime);

    double NkS = Arm.armMass * calculateAccelerationGain(startTime, endTime);
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

  public void teleopArmControls(double extension, double reduction, double rotation) /*double verticalSpeed, double telescopingSpeed
                                                                              * , boolean opengripper, boolean
                                                                              * closegripper, boolean stillgripper1,
                                                                              * boolean stillgripper2
                                                                              */ {
    //rotationMotor.set(rotation);
    if (extension > 0){
      extensionMotor.set(extension);
    }
    else if (reduction < 0){
      extensionMotor.set(reduction);
    }

    rotationMotor.set(rotation);

    /*
     * if (opengripper == true) {
     * gripperMotor.set(1); //speed = positive: to open
     * }
     * if (closegripper == true) {
     * gripperMotor.set(-1); //speed = negative: to close
     * }
     * if (stillgripper1 == true) {
     * gripperMotor.set(0); //speed = 0: to keep gripper closed
     * }
     * if (stillgripper2 == true) {
     * gripperMotor.set(0); //speed = 0: to keep gripper opened
     * }
     */
  }

  // WARNING: THIS HAS A LOT OF OPPURTUNITY TO GO WRONG AND IS VERY IMPRECISE, BUT
  // IT'S

  // button x pressed
  public CommandBase returnToFloor() { // RETRACT BEFORE VERTICAL DOWNWARD MOVEMENT
    return runOnce(
        () -> {
          /* one-time action goes here */
          // using encoders or timer to measure distance needed
          armTime.reset();
          armPosition = ArmPosition.GROUND;
        });
  }

  // button a pressed
  public CommandBase lowGoal() { // RETRACT BEFORE VERTICAL DOWNWARD MOVEMENT
    return runOnce(
        () -> {
          armTime.reset();
          armTime.start();

          if (armPosition == ArmPosition.GROUND) {
            upArm(ArmConstants.floorToLow);
          } else if (armPosition == ArmPosition.MEDIUM) {
            downArm(ArmConstants.midToLow);
          } else if (armPosition == ArmPosition.HIGH) {
            downArm(ArmConstants.highToLow);
          }

          armPosition = ArmPosition.LOW;
        });
  }

  // button b pressed
  public CommandBase midGoal() { // RETRACT BEFORE VERTICAL DOWNWARD MOVEMENT
    return runOnce(
        () -> {
          /* one-time action goes here */
          armTime.reset();
          armTime.start();

          if (armPosition == ArmPosition.GROUND) {
            upArm(ArmConstants.floorToMid);
          } else if (armPosition == ArmPosition.LOW) {
            downArm(ArmConstants.lowToMid);
          } else if (armPosition == ArmPosition.HIGH) {
            downArm(ArmConstants.highToMid);
          }

          armPosition = ArmPosition.MEDIUM;
        });
  }

  // button y pressed
  public CommandBase highGoal() { // RETRACT BEFORE VERTICAL DOWNWARD MOVEMENT
    return runOnce(
        () -> {
          /* one-time action goes here */
          armTime.reset();
          armTime.start();

          if (armPosition == ArmPosition.GROUND) {
            upArm(ArmConstants.floorToHigh);
          } else if (armPosition == ArmPosition.LOW) {
            downArm(ArmConstants.lowToHigh);
          } else if (armPosition == ArmPosition.MEDIUM) {
            downArm(ArmConstants.midToHigh);
          }

          armPosition = ArmPosition.HIGH;
        });
  }

  public void upArm(double timeToPos) {
    while (armTime.get() < timeToPos) {
      rotationMotor.set(ArmConstants.defaultArmSpeed);
    }

    rotationMotor.stopMotor();
  }

  public void downArm(double timeToPos) {
    while (armTime.get() < timeToPos) {
      rotationMotor.set(-ArmConstants.defaultArmSpeed);
    }
    rotationMotor.stopMotor();
  }

  public CommandBase openGripper() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          gripperMotor.set(ArmConstants.defaultGripperSpeed);
        });
  }

  public CommandBase closeGripper() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          gripperMotor.set(-ArmConstants.defaultGripperSpeed);
        });
  }

  public CommandBase stillGripper() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          gripperMotor.stopMotor();
        });
  }
}
