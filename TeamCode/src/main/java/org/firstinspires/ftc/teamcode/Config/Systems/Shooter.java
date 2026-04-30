package org.firstinspires.ftc.teamcode.Config.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shooter {

    private final DcMotorEx motor;
    private final DcMotorEx motor1;
    private final Servo hood;

    public static double targetVelocity = 0;
    private boolean activated = false;

    public static double kS = 0.04; // De compensare
    public static double kV = 0.00045; // 0 -> Target
    public static double kA = 0.00005;
    public static double kP = 0.004;
    public static double kI = 0.0001;

    public static double CLOSE_HOOD = 0.95;
    public static double CLOSE_VELOCITY = 1000;
    private static final double INTEGRAL_MAX = 0.3;

    private double lastVelocity = 0;
    private double previousVelocity = 0;
    private double currentVelocity = 0;
    private double integralSum = 0;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "Jos");
        motor1 = hardwareMap.get(DcMotorEx.class, "Sus");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(CLOSE_HOOD);
    }

    public void setPosition(double position) {
        hood.setPosition(position);
    }

    public void setTarget(double velocity) {
        targetVelocity = velocity;
        integralSum = 0;
        activated = true;
    }

    public void off() {
        activated = false;
        motor.setPower(0);
        motor1.setPower(0);
    }

    public void idle() {
        setTarget(300);
        activated = true;
    }

    public double getVelocity() {
        double rawVel = motor1.getVelocity();
        double filtered = 0.7 * rawVel + 0.3 * lastVelocity;
        lastVelocity = filtered;
        return filtered;
    }

    public double getTarget() {
        return targetVelocity;
    }

    public void periodic() {
        if (!activated) return;

        currentVelocity = getVelocity();
        double error = targetVelocity - currentVelocity;
        double accel = currentVelocity - previousVelocity;
        previousVelocity = currentVelocity;

        integralSum = MathFunctions.clamp(
                integralSum + error,
                -INTEGRAL_MAX / kI,
                INTEGRAL_MAX / kI
        );

        double power = kS * Math.signum(targetVelocity)
                + kV * targetVelocity
                + kA * accel
                + kP * error
                + kI * integralSum;

        power = MathFunctions.clamp(power, 0, 1);

        motor.setPower(power);
        motor1.setPower(power);
    }

    public boolean atTarget() {
        return Math.abs(targetVelocity - currentVelocity) < 50;
    }

    public void shootDistance(double distanceCm) { //Linear interpolation table
        double targetV = 0.0404 * distanceCm * distanceCm - 2.0126 * distanceCm + 1042.35;
        double hoodPos = 0.00232 * distanceCm + 0.548;

        if (targetV > 1600) targetV = 1600;
        if (targetV < 900) targetV = 900;

        if (hoodPos > 0.9) hoodPos = 0.9;
        if (hoodPos < 0.67) hoodPos = 0.67;

        setTarget(targetV);
        setPosition(hoodPos);
    }

    public void close() {
        setTarget(CLOSE_VELOCITY);
        setPosition(CLOSE_HOOD);
    }
    public void far() {
        setTarget(2000);
        setPosition(0.6);
    }
    public CommandBuilder ShootClose()
    {
        return Commands.instant(this::close);
    }
    public CommandBuilder ShootFar()
    {
        return Commands.instant(this::far);

    }
}