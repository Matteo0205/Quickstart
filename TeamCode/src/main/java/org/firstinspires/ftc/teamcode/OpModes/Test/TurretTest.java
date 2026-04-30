package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Systems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {

    Follower follower;
    Turret turret;

    Pose targetPose = new Pose(0, 140, 0);

    @Override
    public void runOpMode() {

        turret = new Turret(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        follower.startTeleopDrive();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            if (gamepad1.a && !turret.activated) {
                turret.activate();
            }

            if (gamepad1.b && turret.activated) {
                turret.idle();
            }

            if (gamepad1.x) {
                targetPose = new Pose(
                        follower.getPose().getX() + 15,
                        follower.getPose().getY() + 15
                );
            }

            Pose robotPose = follower.getPose();

            if (turret.activated) {
                turret.faceWithVelocityCompensation(targetPose, robotPose, follower.getVelocity());
                turret.update();
            }

            telemetry.addData("--- Controls ---", "");
            telemetry.addData("[A]", "Activate turret");
            telemetry.addData("[B]", "Idle turret");
            telemetry.addData("[X]", "Set target to robot +15/+15");
            telemetry.addData("Robot X", robotPose.getX());
            telemetry.addData("Robot Y", robotPose.getY());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.getHeading()));
            telemetry.addData("Target Angle", turret.getTargetAngleDeg());
            telemetry.addData("Current Angle", turret.getCurrentAngleDegPublic());
            telemetry.update();
        }

        turret.idle();
    }
}