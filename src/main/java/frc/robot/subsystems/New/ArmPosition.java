package frc.robot.subsystems.New;

public enum ArmPosition {
    GROUND,
    LOW,
    MEDIUM,
    HIGH,
    BAR,
    STARTING;

    public static final double groundNumericPosition = 0;
    public static final double lowNumericPosition = 1;
    public static final double mediumNumericPosition = 2;
    public static final double highNumericPosition = 3;
    public static final double barNumericPosition = 4;
    public static final double startingNumericPosition = 4;

    public static double numericPosition(ArmPosition pos) {
        switch(pos) {
        case LOW:
            return ArmPosition.lowNumericPosition;
        case MEDIUM:
            return ArmPosition.mediumNumericPosition;
        case HIGH:
            return ArmPosition.highNumericPosition;
         case BAR:
            return ArmPosition.barNumericPosition;
        case STARTING:
            return ArmPosition.startingNumericPosition;
        default:
            return ArmPosition.groundNumericPosition;
        }
    }
}
