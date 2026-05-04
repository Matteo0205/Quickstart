package org.firstinspires.ftc.teamcode.Config.Util;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.Systems.Gate;
import org.firstinspires.ftc.teamcode.Config.Systems.Intake;
import org.firstinspires.ftc.teamcode.Config.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Systems.Transfer;
import org.firstinspires.ftc.teamcode.Config.Systems.Turret;

public class Robot {

    private final Intake intake;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Turret turret;
    private final Gate gate;

    private ShootingState state = ShootingState.IDLE;

    private enum ShootingState {
        IDLE,
        SPINNING_UP,
        SHOOTING
    }

    public Robot(HardwareMap hardwareMap) {
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);

        turret.init();
    }

    public void periodic() {
        shooter.periodic();
        turret.update();
        handleShooter();
    }

    public void closeShooter(double target, double pos) {
        shooter.setTarget(target);
        shooter.setPosition(pos);
    }

    public void handleShooter() {
        switch (state) {
            case IDLE:
                shooter.off();
                intake.stop();
                transfer.stop();
                gate.close();
                break;

            case SPINNING_UP:
                shooter.close();
                gate.open();

                if (shooter.atTarget() && turret.isAtTarget()) {
                    state = ShootingState.SHOOTING;
                }
                break;

            case SHOOTING:
                transfer.on();
                intake.on();
                break;
        }
    }

    public void startShooting() {
        state = ShootingState.SPINNING_UP;
    }

    public void stopShooting() {
        state = ShootingState.IDLE;
    }

    public void intakeSpinIn() {
        intake.on();
        transfer.on();
    }

    public void intakeSpinOut() {
        intake.reverse();
        transfer.reverse();
    }

    public void intakeStop() {
        if (state != ShootingState.SPINNING_UP) {
            intake.stop();
            transfer.stop();
        }
    }

    public void stopAll() {
        stopShooting();
        intakeStop();
    }

    public double getDistance(Pose robotPose, Pose targetPose) {
        return Math.hypot(
                targetPose.getX() - robotPose.getX(),
                targetPose.getY() - robotPose.getY()
        );
    }

    public void turretAim(Pose robotPose, Pose targetPose, Vector robotVelocity) {
        turret.faceWithVelocityCompensation(robotPose, targetPose, robotVelocity);
    }

    public double getTurretAngle() {
        return turret.getCurrentAngleDegPublic();
    }

    public CommandBuilder startShootingCommand() {
        return Commands.instant(this::startShooting);
    }

    public CommandBuilder stopShootingCommand() {
        return Commands.instant(this::stopShooting);
    }

    public CommandBuilder periodicCommand() {
        return Commands.instant(this::periodic);
    }

    public CommandBuilder handleShooterCommand() {
        return Commands.instant(this::handleShooter);
    }

    public CommandBuilder intakeSpinInCommand() {
        return Commands.instant(this::intakeSpinIn);
    }

    public CommandBuilder intakeSpinOutCommand() {
        return Commands.instant(this::intakeSpinOut);
    }

    public CommandBuilder intakeStopCommand() {
        return Commands.instant(this::intakeStop);
    }

    public CommandBuilder stopAllCommand() {
        return Commands.instant(this::stopAll);
    }

    public CommandBuilder turretAimCommand(Pose robotPose, Pose targetPose, Vector robotVelocity) {
        return Commands.instant(() -> turretAim(robotPose, targetPose, robotVelocity));
    }

    public CommandBuilder closeShootCommand(double target, double pos) {
        return instant(() -> closeShooter(target, pos));
    }

    public CommandBuilder autoShootCommand() {
        return sequential(
                Commands.waitUntil(shooter::atTarget),
                Commands.instant(gate::open),
                intakeSpinInCommand(),
                Commands.waitMs(2000),
                Commands.instant(gate::close)
        );
    }
}