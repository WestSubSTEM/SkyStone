package org.firstinspires.ftc.teamcode;
// Constants to be used by all STEMper Fi programs
public class StemperFiConstants {
    // Limits for the VEX_EDR_393 motors if you set value outside of limits motor will not move
    public static final double VEX_EDR_393_MIN = 0.1;
    public static final double VEX_EDR_393_OFF = 0.5;
    public static final double VEX_EDR_393_MAX = 0.9;
    public static final double VEX_EDR_393_MAX_RANGE = VEX_EDR_393_MAX - VEX_EDR_393_OFF;
    public static final double VEX_EDR_393_MIN_RANGE = VEX_EDR_393_OFF - VEX_EDR_393_MIN;

    // These servos positions were calcuated using the RevHub software
    // which let us set discreate values and determine the correct one.
    public static final double FOUNDATION_SERVO_LEFT_UP = 0.86;
    public static final double FOUNDATION_SERVO_LEFT_DOWN = 0.18;

    public static final double FOUNDATION_SERVO_RIGHT_UP = 0.08;
    public static final double FOUNDATION_SERVO_RIGHT_DOWN = 0.75;

    public static final double SKYSTONE_SERVO_OPEN = 0.75;
    public static final double SKYSTONE_SERVO_CLOSE = 0.0;

    public static final double EXTENSION_SERVO_IN = 1.0;
    public static final double EXTENSION_SERVO_OUT = 0.0;

    public static final int[] LIFT_POSITIONS = new int[] {0, 1391, 3245, 5099, 6953, 8807, 10662, 12516, 14370, 16224, 18078, 19933};

    // How many encoder ticks to move forward/backwards 1 cm
    public static final int TICKS_PER_CM = 22;
    // Hom many encoder ticks to slide left/right 1 cm
    public static final int SLIDE_TICKS_PER_CM = 25;
}
