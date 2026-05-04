package org.firstinspires.ftc.teamcode.OpModes.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Util.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Configurable
@TeleOp(name = "Albastru", group = "TeleOp")
public class Albastru extends OpMode {

    private Robot robot;
    private Follower follower;
    private ElapsedTime timer;

    private boolean previousY;
    private boolean previousB;
    private boolean rumbled;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));

        timer = new ElapsedTime();

        previousY = false;
        previousB = false;
        rumbled = false;
    }

    @Override
    public void start() {
        timer.reset();
        follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        boolean currentY = gamepad1.y;
        boolean currentB = gamepad1.b;

        if (currentY && !previousY) {
            robot.startShooting();
        }

        if (currentB && !previousB) {
            robot.stopAll();
        }

        previousY = currentY;
        previousB = currentB;

        if (gamepad1.right_trigger > 0.1) {
            robot.intakeSpinIn();
        } else if (gamepad1.left_trigger > 0.1) {
            robot.intakeSpinOut();
        } else {
            robot.intakeStop();
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        if (!rumbled && timer.seconds() >= 100) {
            gamepad1.rumble(1000);
            rumbled = true;
        }

        follower.update();
        robot.periodic();
    }
}