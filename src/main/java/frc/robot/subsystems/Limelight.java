package frc.robot.subsystems;
/*
 * Imports section lol
 * 
 * 
 * if returning latency time variables becomes an issue, reconsider reinstating force test method (god help me)
 */
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Class for limelight net. table
 * 
 * @LUIGIBLASQUEZ
 */
public class Limelight extends SubsystemBase{
    private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private static NetworkTable table = tableInstance.getTable("limelight");
    //network table testing status
    private static NetworkTableEntry timingTestEntry = table.getEntry("TIMING_TEST_ENTRY");
    private static boolean timingTestEntryValue = false;
    public static final long MAX_UPDATE_TIME = 100_000; // microseconds; 0.1 seconds
    /*
     * SmartDashboard initializing
     * 
     * 
     */
    public static final boolean POST_TO_SMART_DASHBOARD = true;

    //force test variables
    boolean timingTestEntryValue2 = !timingTestEntryValue;
    // timingTestEntry.forceSetBoolean(timingTestEntryValue);

    static long currentTime = timingTestEntry.getLastChange();
    //temp
    // The pipeline’s latency contribution (ms)
    public static final double IMAGE_CAPTURE_LATENCY = 11;
    private static NetworkTableEntry latencyEntry = table.getEntry("tl");
    //temp
    static long lastUpdate = latencyEntry.getLastChange();
    //connected variables
    static long timeDifference = currentTime - lastUpdate;
    static boolean connected = timeDifference < MAX_UPDATE_TIME;

    //returns if smart dash. is connected
    public static boolean isConnected() {
        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Limelight Connected", connected);
                //SmartDashboard.putNumberConnection("Limelight Time Difference", timeDifference);
            SmartDashboard.putNumber("Limelight Time Difference", timeDifference);
    }
    return connected;
    }

    /*
     * DISPLAY VARIABLES
     * not final type if someone would like an edit of aspect ratio etc
     *
     */
    public static double DEFAULT_TARGET_HEIGHT_THRESHOLD = 6;
    public static double DEFAULT_MIN_ASPECT_RATIO = 1.0;
    public static double DEFAULT_MAX_ASPECT_RATIO = 3.0;
    public static double DEFAULT_ANGLE_THRESHOLD = 25;

    //limelight target tracker

    public static boolean hasValidTarget(double targetHeightThreshold, double minRatio, double maxRatio,double angleThreshold)
        {
             {
                return hasAnyTarget() 
                & hasValidHeight(targetHeightThreshold) 
                & hasValidBlueAspectRatio(minRatio, maxRatio)
                & hasValidBlueOrientation(angleThreshold);
                }
            }
    //check 2
    public static boolean hasValidTarget() {
        if(SmartDashboard.getBoolean("CV_FILTER_OVERRIDE", false)) {
            return hasValidTarget(
                SmartDashboard.getNumber("HEIGHT_THRESHOLD",
                 DEFAULT_TARGET_HEIGHT_THRESHOLD),
                SmartDashboard.getNumber("MIN_ASPECT_RATIO",
                 DEFAULT_MIN_ASPECT_RATIO),
                SmartDashboard.getNumber("MAX_ASPECT_RATIO",
                 DEFAULT_MAX_ASPECT_RATIO),
                SmartDashboard.getNumber("SKEW_THRESHOLD",
                 DEFAULT_ANGLE_THRESHOLD));
        }
        return hasValidTarget(
            DEFAULT_TARGET_HEIGHT_THRESHOLD,
            DEFAULT_MIN_ASPECT_RATIO,
            DEFAULT_MAX_ASPECT_RATIO,
            DEFAULT_ANGLE_THRESHOLD);
    }
    /*
     * VALID TARGET ASPECTS
     * 
     * 
     * 
     */
    private static NetworkTableEntry validTargetEntry = table.getEntry("tv");

    public static boolean hasAnyTarget() {
        boolean validTarget = validTargetEntry.getDouble(0) > 0.5;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Target", validTarget);
        }

        return validTarget;
    }
    public static boolean hasValidHeight(double targetHeightThreshold) {
        boolean validHeight = getTargetYAngle() < targetHeightThreshold;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Height", validHeight);
        }

        return validHeight;
    }
    public static boolean hasValidBlueAspectRatio(double minRatio, double maxRatio) {
        double aspectRatio = getHorizontalSidelength() / getVerticalSidelength();
        boolean validRatio = aspectRatio > minRatio && aspectRatio < maxRatio;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Ratio", validRatio);
            SmartDashboard.putNumber("Aspect Ratio", aspectRatio);
        }

        return validRatio;
    }
    public static boolean hasValidBlueOrientation(double angleThreshold) {
        double skew = Math.abs(getTargetSkew());
        boolean validOrientation = Math.min(skew, 90.0 - skew) <= angleThreshold;

        if (POST_TO_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Valid Skew", validOrientation);
            SmartDashboard.putNumber("Skew Value", Math.min(skew, 90.0 - skew));
        }

        return validOrientation;
    }
    // horiz. offsets from crosshair to target
    public static final double MIN_X_ANGLE = -27;
    public static final double MAX_X_ANGLE = 27;
    public static final double X_ANGLE_SHIFT = -1.5;
    private static NetworkTableEntry xAngleEntry = table.getEntry("tx");
    // Vert. offset from crosshair to target
    public static final double MIN_Y_ANGLE = -20.5;
    public static final double MAX_Y_ANGLE = 20.5;
    private static NetworkTableEntry yAngleEntry = table.getEntry("ty");
    /*
     * Camera get methods (help)
     * 
     * 
     * 
     * 
     */
    public static double getTargetXAngle() {

        double X_SHIFT = SmartDashboard.getNumber("X_SHIFT", 1000);
        if(X_SHIFT > 694) SmartDashboard.putNumber("X_SHIFT", X_ANGLE_SHIFT);
        return xAngleEntry.getDouble(0) + X_SHIFT;
    }

    //vert. angle of target
    public static double getTargetYAngle() {
        return yAngleEntry.getDouble(0);
    }

    // target area; range 0-100%
    public static final double MIN_TARGET_AREA = 0;
    public static final double MAX_TARGET_AREA = 1;
    private static NetworkTableEntry targetAreaEntry = table.getEntry("ta");

    //% of the screen the target will take up; 0-1 scale
    public static double getTargetArea() {
        return Math.min(targetAreaEntry.getDouble(0) / 100.0, 1);
    }

    // skew from -90-0 degs.
    public static final double MIN_SKEW = -90;
    public static final double MAX_SKEW = 0;
    private static NetworkTableEntry targetSkewEntry = table.getEntry("ts");

    //returns target skew
    public static double getTargetSkew() {
        return targetSkewEntry.getDouble(0);
    }

    //return the latency of limelight information (damn slow lol)
    public static double getLatencyMs() {
        return latencyEntry.getDouble(0) + IMAGE_CAPTURE_LATENCY;
    }

    // information returned from the func.
    public static final double MIN_SIDE_LENGTH = 0;
    public static final double MAX_SIDE_LENGTH = 320;

    private static NetworkTableEntry shortestSideLengthEntry = table.getEntry("tshort");

    //returns shortest side length of the target in pixels
    public static double getShortestSidelength() {
        return shortestSideLengthEntry.getDouble(0);
    }

    private static NetworkTableEntry longestSideLengthEntry = table.getEntry("tlong");

    //return longest side length of the target in pixels
    public static double getLongestSidelength() {
        return longestSideLengthEntry.getDouble(0);
    }

    private static NetworkTableEntry horizontalSideLengthEntry = table.getEntry("thor");

    //return horiz. side length of target in pixels
    public static double getHorizontalSidelength() {
        return horizontalSideLengthEntry.getDouble(0);
    }

    private static NetworkTableEntry verticalSideLengthEntry = table.getEntry("tvert");

    //return vert. side length of target in pixels
    public static double getVerticalSidelength() {
        return verticalSideLengthEntry.getDouble(0);
    }
    /*
     * cam. controllers
     * 
     * 
     * 
     */
        public enum LEDMode {
            PIPELINE(0), // Use LED mode set in pipeline
            FORCE_OFF(1), // Force LEDs off
            FORCE_BLINK(2), // Force LEDs to blink
            FORCE_ON(3); // Force LEDs on
        
    
            LEDMode(int value) {
                this.val = value;
            }
    
            public int getCodeValue() {
                return val;
            }
    
            private int val;
        }    
    
    
        private static NetworkTableEntry LEDModeEntry = table.getEntry("ledMode");
    
        // led mode limelight is set too
        public final void setLEDMode(LEDMode mode) {
            LEDModeEntry.setNumber(mode.getCodeValue());
        }
    
        // camera mode stuff
        public enum CamMode {
            VISION(0), // just use as a camera view
            DRIVER(1); // use for driving too
    
            CamMode(int value) {
                this.val = value;
            }
    
            public int getCodeValue() {
                return val;
            }
    
            private int val;
        };
    
        private static NetworkTableEntry camModeEntry = table.getEntry("camMode");
    
        //pick a cam mode to set limelight too
        public static void setCamMode(CamMode mode) {
            camModeEntry.setNumber(mode.getCodeValue());
        }
    
        // pipeline stuff
        private static NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    
        // pipeline the limelight is set too
        public static void setPipeline(int pipeline) {
            if (pipeline >= 0 && pipeline <= 9) {
                pipelineEntry.setNumber(pipeline);
            }
        }
    
        private static NetworkTableEntry getPipelineEntry = table.getEntry("getpipe");
    
        public static double getPipeline() {
            return getPipelineEntry.getDouble(0);
        }
    
        // camera stream
        public enum CameraStream { 
            STANDARD(0), // limelight w/ secondary camera
            PIP_MAIN(1), // secondary cam inside limelight main cam
            PIP_SECONDARY(2); // display limelight inside the limelight camera
    
            CameraStream(int value) {
                this.val = value;
            }
    
            public int getCodeValue() {
                return val;
            }
    
            private int val;
        };
    
        private static NetworkTableEntry CameraStreamEntry = table.getEntry("stream");
    
        //camera stream limelight is viewed on
        public static void setCameraStream(CameraStream stream) {
            CameraStreamEntry.setNumber(stream.getCodeValue());
        }
    
        // snapshot mode
        public enum SnapshotMode {
            STOP(0), //stop taking snapshots
            TWO_PER_SECOND(1);
    
            SnapshotMode(int value) {
                this.val = value;
            }
    
            public int getCodeValue() {
                return val;
            }
    
            private int val;
        };
    
        private static NetworkTableEntry SnapshotModeEntry = table.getEntry("snapshot");

        public static void setSnapshotMode(SnapshotMode mode) {
            SnapshotModeEntry.setNumber(mode.getCodeValue());
    }
}