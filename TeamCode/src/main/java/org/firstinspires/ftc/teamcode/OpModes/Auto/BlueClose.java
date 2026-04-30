package org.firstinspires.ftc.teamcode.OpModes.Auto;


import static com.pedropathing.ivy.Scheduler.schedule;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Config.Util.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static com.pedropathing.ivy.groups.Groups.sequential;


@Config
@Autonomous(name = "Blue Close", group = "Auto")
public class BlueClose extends OpMode {
    Follower follower;
    Robot robot;
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.585, 118.439),

                                    new Pose(58.732, 73.854)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(176))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.732, 73.854),

                                    new Pose(18.878, 58.927)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.878, 58.927),

                                    new Pose(58.756, 73.707)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.756, 73.707),
                                    new Pose(26.927, 49.171),
                                    new Pose(11.610, 61.756)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.610, 61.756),

                                    new Pose(50.683, 83.683)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.683, 83.683),

                                    new Pose(22.561, 83.146)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.561, 83.146),

                                    new Pose(46.366, 114.683)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }
    public enum States{
        GOTOSHOOTPRELOAD,
        SHOOTPRELOAD,
        GOTOINTAKEFIRST,
        GOTOFIRSTSHOOT,
        SHOOTFIRST,
        GOTOGATEINTAKEFIRST,
        GOTOSHOOTGATEFIRST,
    }
    public void stateMachine()
    {

    }
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap);
        Paths paths = new Paths(follower);
    }

    @Override
    public void loop() {

    }
}
