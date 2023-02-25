package frc.robot.subsystems.New;

public enum ArmPosition {
    GROUND,
    LOW,
    MEDIUM,
    HIGH;

    public static final double groundNumericPosition = 0;
    public static final double lowNumericPosition = 0;
    public static final double mediumNumericPosition = 0;
    public static final double highNumericPosition = 0;

    public static double numericPosition(ArmPosition pos) {
        switch(pos) {
        case LOW:
            return ArmPosition.lowNumericPosition;
        case MEDIUM:
            return ArmPosition.mediumNumericPosition;
        case HIGH:
            return ArmPosition.highNumericPosition;
        default:
            return ArmPosition.groundNumericPosition;
        }
    }
}
