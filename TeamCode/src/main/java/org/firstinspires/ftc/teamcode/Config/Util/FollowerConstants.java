package org.firstinspires.ftc.teamcode.Config.Util;

import org.opencv.core.Mat;

public final class FollowerConstants {
    public static double X = 0.0;
    public static double Y = 0.0;
    public static double HEADING = 0.0;

    private FollowerConstants() {}

    public static void set(double x, double y, double heading) {
        X = x;
        Y = y;
        HEADING = heading;
    }

    public static void reset() {
        X = 0.0;
        Y = 0.0;
        HEADING = Math.PI / 2;
    }
}
