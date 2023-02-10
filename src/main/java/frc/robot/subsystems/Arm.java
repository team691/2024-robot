package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.Duration;
import java.time.Instant;

public class Arm extends SubsystemBase {
    // Mass measured in grams
    public static final double armMass = 0.0;

    // Lengths measured in <some unit>
    public static final double maxArmLength = 0.0;
    public static final double minArmLength = 0.0;
    
    public static final double initialArmLength = 0.0;
    public static final double initialArmAngle = 0.0;

    public static final double initialArmVelocity = 0.0;

    private ArmFeedforward feedforward;

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
        this.feedforward = new ArmFeedforward(Arm.initialArmStaticGain, Arm.initialArmGravityGain, Arm.initialArmVelocityGain, Arm.initialArmAccelerationGain);

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
     * @return Robot's current arm angle
     */
    public double getCurrentArmAngle() {
        return this.currentArmAngle;
    }

    /**
     * Accesses robot's current arm length
     * @return Robot's current arm length
     */
    public double getCurrentArmLength() {
        return this.currentArmLength;
    }

    /**
     * Accesses robot's last arm angle
     * @return Robot's last arm angle
     */
    public double getLastArmAngle() {
        return this.lastArmAngle;
    }

    /**
     * Accesses robot's last arm length
     * @return Robot's last arm length
     */
    public double getLastArmLength() {
        return this.lastArmLength;
    }

    /**
     * Sets robot's arm angle
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
     * @param newLength Length to set arm to
     */
    public void setArmLength(double newLength) {
        this.lastArmLength = this.currentArmLength;
        this.currentArmLength = newLength;

        updateFeedforward();
        updateArmHardware();
    }

    // TODO: Calculate static gain
    /**
     * Calculates the static gain of the robot arm from one moment to another
     * @param startTime The start moment
     * @param endTime The end moment
     * @return Static gain
     */
    private double calculateStaticGain(Instant startTime, Instant endTime) {
        return 0.0;
    }

    /**
     * Calculates the gravitational gain of the robot arm from one moment to another
     * @param startTime The start moment
     * @param endTime The end moment
     * @return Gravitational gain
     */
    private double calculateGravityGain(Instant startTime, Instant endTime) {
        // Dividing by 1 million to convert m/s^2 to m/ms^2
        final double gravitationalAccelerationMillis = 9.8 / 1_000_000;
        final double gravitationalForce = Arm.armMass * gravitationalAccelerationMillis;
        final double deltaTime = (double) Duration.between(startTime, endTime).toMillis();
        final double gravity = gravitationalForce / deltatime;
        return gravity;
    }

    /**
     * Calculates the velocity gain of the robot arm between two moments
     * @param startTime The start moment
     * @param endTime The end moment
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
     * @param startTime The start moment
     * @param endTime The end moment
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
        // TODO: Update the arm hardware
    }
}