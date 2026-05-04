package org.firstinspires.ftc.teamcode.Config.Util;

public final class TurretState {
    public static double ANGLE = 0.0;
    public static boolean HAS_ANGLE = false;

    private TurretState() {}

    public static void save(double angleDeg) {
        ANGLE = angleDeg;
        HAS_ANGLE = true;
    }

    public static void reset() {
        ANGLE = 0.0;
        HAS_ANGLE = false;
    }
}