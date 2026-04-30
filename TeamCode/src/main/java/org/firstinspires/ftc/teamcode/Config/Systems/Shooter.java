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
import com.seattlesolvers.solverslib.util.LUT;

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

    public void ShootDistances(double distance) {
        LUT<Double, Double> speeds = new LUT<Double, Double >();
        LUT<Double, Double> HoodAngle = new LUT<Double, Double>();

        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(10.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        speeds.add(100.0, 1200.0);
        double v = speeds.get(distance); // generezi ecuatia pentru distanta

        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        HoodAngle.add(100.0, 0.1);
        double a = HoodAngle.get(distance); // generezi ecuatia pentru distanta

        if(v > 2200)
        {
            setTarget(2200);
        } else
        {
            setTarget(v);
        }
        if(a > 1)
        {
            setPosition(1);
        } else if (a < 0)
        {
            setPosition(0);
        } else
        {
            setPosition(a);
        }
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