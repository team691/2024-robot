// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.</p>
 */

public final class Constants {
  public static class OperatorConstants {
   // public static final int kDriverControllerPort = 0;
    public static final int kStick1ControllerPort = 0;
    public static final int kStick2ControllerPort = 1;
    public static final int kXboxControllerPort = 2;
  }
  public static class DriveConstants {
    public static final int kFrontLeftMotorID = 1;
    public static final int kRearLeftMotorID = 2;
    public static final int kFrontRightMotorID = 3;
    public static final int kRearRightMotorID = 4;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};

    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    
  /* cycles per revolution; this isn't actually the constants for the variables;
  it's just referenced from the MechanumControllerCommand Example as provided by the WPLib extention on VSCode*/
    public static final double kEncoderCPR = 535.5;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
      // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) /  kEncoderCPR;
    public static final double kRadiusTurn = 5;
  }

//Regarding drive train movement, a positive # value moves "forward" with the bar as the front, and a negative # with the battery as the back
  public static class AutoConstants{
    public static final double kTimeoutSeconds = 3;
    public static final double kDriveDistanceMeters = 2;
    public static final double kDriveSpeed = 0.5;
    public static final double kTurnSpeed = 0.75;
    public static final double kBalancingSpeed = 0.01;
    public static final double kHardTurn = 0.1;
    public static double kAutoDriveDistanceInchesF = 11.0;
    public static double kAutoDriveDistanceInchesB = -190.0;
    public static double kAutoDriveDistanceInchesBalance = -54.0;
    public static double kAutoDriveSpeed = 0.4;
    public static double kAutoBalanceSpeed = 0.6;
    public static double rot;
    public static double rot1;
    public static double rot2;
    public static double extensionTime;
    public static double retractionTime;
  }

  public static class LimelightConstants{
    public static final int LED_ON = 3;
    public static final int LED_OFF = 1;
    public static final int TARGET_PIPELINE = 0;
    public static final int DEFAULT_PIPELINE = 0;
    public static final int DRIVE_PIPELINE = 2;
  }
  
  public static class ArmConstants{
    public static final int intakeMotorID = 8;
    public static final int wristMotorID = 5;
    public static final int rotationMotorID = 7;
    public static final int extensionMotorID = 6;

    // rotation constants for time - needs HEAVY trial and error
    public static final double timeToDown = 2.25;
    public static final double floorToLow = 0;
    public static final double floorToMid = 0;
    public static final double floorToHigh = 0;

    public static final double lowToMid = 0;
    public static final double lowToHigh = 0;
    public static final double midToHigh = 0;

    public static final double midToLow = 0;
    public static final double highToMid = 0;
    public static final double highToLow = 0;

    public static final double lowToFloor = 0;
    public static final double midToFloor = 0;
    public static final double highToFloor = 0;

    public static final double barToHigh = 2.75;
    public static final double barToMid = 3;
    public static final double barToLow = 4;
    
    public static final double defaultExtentionSpeed = 0;
    public static final double defaultRotationSpeed = .25;
    public static final double defaultGripperSpeed = -.4;
    public static final double openGripperTime = 1;
    public static final double closeGripperTime = .5;
    public static final double timeToBar = 1.5;
    public static final double LowerTimeout = 0.5;
    public static final double lowerGripperTime = 0;
    public static final double raiseGripperTime = 0;
  }
  /*public static class GripperConstants{
    public static final int kGripperMotorID = 5;
  } */
}
