package org.firstinspires.ftc.teamcode.Config.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;

import org.firstinspires.ftc.teamcode.Config.Util.TurretState;

@Config
public class Turret {

    public static double timeOfFlight = 0;

    public static double kP = 0.002;
    public static double kD = 0.000001;

    public static double TICKS_PER_REV = 8192.0;
    public static double GEAR_RATIO = 1.0;

    public static double MAX_POWER = 1.0;
    public static double DEAD_ZONE_DEG = 0.5;

    public static double MAX_ANGLE_DEG = 90.0;

    private final CRServo servo1;
    private final CRServo servo2;
    private final CRServoGroup servoGroup;

    private final DcMotor encoderMotor;

    private double targetAngleDeg = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;
    private int encoderOffset = 0;

    public boolean activated = false;

    public Turret(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "ServoTuretaStanga");
        servo2 = hardwareMap.get(CRServo.class, "ServoTuretaDreapta");

        servo2.setInverted(true);
        servoGroup = new CRServoGroup(servo1, servo2);

        encoderMotor = hardwareMap.get(DcMotor.class, "intake");
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = System.nanoTime();
    }

    public void init() {
        if (TurretState.HAS_ANGLE) {
            resumeFromPosition(TurretState.ANGLE);
        } else {
            resetEncoder();
        }
    }

    public void resetEncoder() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffset = 0;
        lastError = 0.0;
        targetAngleDeg = 0.0;
        lastTime = System.nanoTime();
    }

    public void resumeFromPosition(double savedAngleDeg) {
        encoderOffset = (int) ((savedAngleDeg / 360.0) * TICKS_PER_REV * GEAR_RATIO);
        lastError = 0.0;
        targetAngleDeg = savedAngleDeg;
        lastTime = System.nanoTime();
    }

    public void saveCurrentAngle() {
        TurretState.save(getCurrentAngleDeg());
    }

    public void faceWithVelocityCompensation(Pose targetPose, Pose robotPose, Vector robotVelocity) {
        double vx = robotVelocity.getXComponent();
        double vy = robotVelocity.getYComponent();

        double compensatedX = targetPose.getX() - vx * timeOfFlight;
        double compensatedY = targetPose.getY() - vy * timeOfFlight;

        double angleToTarget = Math.atan2(
                compensatedY - robotPose.getY(),
                compensatedX - robotPose.getX()
        );

        double turretAngle = normalizeAngle(angleToTarget - robotPose.getHeading());
        double angleDeg = Math.toDegrees(turretAngle);

        angleDeg = Math.max(-MAX_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, angleDeg));
        targetAngleDeg = angleDeg;
    }

    public void update() {
        if (!activated) {
            setPower(0);
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (dt <= 0) return;

        double currentAngleDeg = getCurrentAngleDeg();
        double error = targetAngleDeg - currentAngleDeg;
        double derivative = (error - lastError) / dt;
        lastError = error;

        if (Math.abs(error) < DEAD_ZONE_DEG) {
            setPower(0);
            return;
        }

        double power = kP * error + kD * derivative;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        setPower(power);
    }

    private void setPower(double power) {
        servoGroup.set(power);
    }

    private double getCurrentAngleDeg() {
        double ticks = encoderMotor.getCurrentPosition() + encoderOffset;
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 360.0;
    }

    public void activate() {
        activated = true;
        lastTime = System.nanoTime();
    }

    public void idle() {
        activated = false;
        targetAngleDeg = 0.0;
        lastError = 0.0;
        setPower(0);
    }

    public boolean isAtTarget() {
        return Math.abs(targetAngleDeg - getCurrentAngleDeg()) < DEAD_ZONE_DEG;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public double getCurrentAngleDegPublic() {
        return getCurrentAngleDeg();
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2.0);
        if (angle <= -Math.PI) angle += Math.PI * 2.0;
        if (angle > Math.PI) angle -= Math.PI * 2.0;
        return angle;
    }
}