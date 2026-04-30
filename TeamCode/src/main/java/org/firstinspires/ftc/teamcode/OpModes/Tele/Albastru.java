package org.firstinspires.ftc.teamcode.OpModes.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Util.FollowerConstants;
import org.firstinspires.ftc.teamcode.Config.Util.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Configurable
@TeleOp(name = "Albastru" , group = "TeleOp")
public class Albastru extends OpMode {
    Robot robot;
    Follower follower;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(new Pose(0, 0 , Math.toRadians(90)));
        timer = new ElapsedTime();
    }
    @Override
    public void start()
    {
        timer.reset();
        follower.startTeleOpDrive(true);
    }
    @Override
    public void loop() {

        if(gamepad1.yWasPressed() )
        {
            robot.startShooting();
        }
        if(gamepad1.bWasPressed())
        {
            robot.stopAll();
        }

        if(gamepad1.right_trigger > 0.1)
        {
            robot.IntakeSpinIn();
        }
        else if(gamepad1.left_trigger > 0.1)
        {
            robot.IntakeSpinOut();
        }
        else
        {
            robot.IntakeStop();
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if(timer.seconds() == 100)
        {
            gamepad1.rumble(1000);
        }
        robot.handleshooter();
        follower.update();
        robot.periodic();
    }
    }


