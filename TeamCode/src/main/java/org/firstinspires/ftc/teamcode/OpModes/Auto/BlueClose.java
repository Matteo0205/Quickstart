package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.Util.Robot;
import org.firstinspires.ftc.teamcode.Config.Util.TurretState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "Blue Close", group = "Auto")
public class BlueClose extends OpMode {

    private Follower follower;
    private Robot robot;
    private Paths paths;

    public static class Paths {
        public final PathChain path1;
        public final PathChain path2;
        public final PathChain path3;
        public final PathChain path4;
        public final PathChain path5;
        public final PathChain path6;
        public final PathChain path7;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(16.585, 118.439),
                            new Pose(58.732, 73.854)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(176))
                    .build();

            path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.732, 73.854),
                            new Pose(18.878, 58.927)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.878, 58.927),
                            new Pose(58.756, 73.707)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.756, 73.707),
                            new Pose(26.927, 49.171),
                            new Pose(11.610, 61.756)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.610, 61.756),
                            new Pose(50.683, 83.683)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(50.683, 83.683),
                            new Pose(22.561, 83.146)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            path7 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.561, 83.146),
                            new Pose(46.366, 114.683)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(16.585, 118.439));

        TurretState.reset();

        robot = new Robot(hardwareMap);
        paths = new Paths(follower);

        schedule(
                Commands.infinite(robot::periodic),
                sequential(
                        robot.closeShootCommand(1000, 0.8),
                        PedroCommands.follow(follower, paths.path1)
                                .with(
                                        Commands.waitUntil(() -> follower.getCurrentTValue() >= 0.5)
                                                .then(robot.autoShootCommand())
                                ),
                        PedroCommands.follow(follower, paths.path2)
                )
        );
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        TurretState.save(robot.getTurretAngle());
    }
}