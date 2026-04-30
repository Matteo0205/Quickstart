package org.firstinspires.ftc.teamcode.Config.Util;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.pedropathing.math.Vector;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;


import org.firstinspires.ftc.teamcode.Config.Systems.Intake;
import org.firstinspires.ftc.teamcode.Config.Systems.Gate;
import org.firstinspires.ftc.teamcode.Config.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Systems.Transfer;
import org.firstinspires.ftc.teamcode.Config.Systems.Turret;

import java.time.Instant;

public class Robot {
    private final Intake intake;
    private final Shooter shooter;
    private final Transfer transfer;
    private final Turret turret;
    private final Gate gate;
    enum ShootingState {
        IDLE,
        SPINNING_UP,
        SHOOTING
    }
    ShootingState state = ShootingState.IDLE;
    public Robot(HardwareMap HwMap) {
        shooter = new Shooter(HwMap);
        transfer = new Transfer(HwMap);
        turret = new Turret(HwMap);
        gate = new Gate(HwMap);
        intake = new Intake(HwMap);
    }
    public void periodic()
    {
        shooter.periodic();
        turret.update();
    }
    public void handleshooter()
    {
        switch(state)
        {
            case IDLE:
                shooter.off();
                intake.stop();
                transfer.stop();
                gate.close();
                break;
            case SPINNING_UP:
                shooter.close();
                gate.open();
                if(shooter.atTarget() && turret.isAtTarget())
                {
                    state = ShootingState.SHOOTING;
                }
                break;
            case SHOOTING:
                transfer.on();
                intake.on();
                break;
        }
    }
    public void startShooting()
    {
        state = ShootingState.SPINNING_UP;
    }
    public void stopShooting()
    {
        state = ShootingState.IDLE;
    }
    public void IntakeSpinIn()
    {
        intake.on();
        transfer.on();
    }
    public void IntakeSpinOut()
    {
        intake.reverse();
        transfer.reverse();
    }
    public void IntakeStop()
    {
        if(state != ShootingState.SPINNING_UP) {
            intake.stop();
            transfer.stop();
        }
    }
    public void stopAll()
    {
        stopShooting();
        IntakeStop();
    }
    public double getDistance(Pose robotPose , Pose targetPose)
    {
        double distance = 0;
        distance = Math.hypot(targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
        return distance;
    }
    public void TurretAim(Pose robotPose , Pose targetPose ,Vector robotVelocity)
    {
        turret.faceWithVelocityCompensation(robotPose , targetPose , robotVelocity);
    }

    /** IVY **/
    public CommandBuilder StartShootingCommand() {
        return Commands.instant(this::startShooting);
    }
    public CommandBuilder StopShootingCommand() {
        return Commands.instant(this::stopShooting);
    }
    public CommandBuilder PeriodicCommand() {
        return Commands.instant(this::periodic);
    }
    public CommandBuilder HandleShooterCommand() {
        return Commands.instant(this::handleshooter);
    }
    public CommandBuilder IntakeSpinInCommand() {
        return Commands.instant(this::IntakeSpinIn);
    }
    public CommandBuilder IntakeSpinOutCommand() {
        return Commands.instant(this::IntakeSpinOut);
    }
    public CommandBuilder IntakeStopCommand() {
        return Commands.instant(this::IntakeStop);
    }
     public CommandBuilder StopAllCommand() {
        return Commands.instant(this::stopAll);
    }
    public CommandBuilder TurretAimCommand(Pose robotPose , Pose targetPose ,Vector robotVelocity) {
        return Commands.instant(() -> TurretAim(robotPose, targetPose, robotVelocity));
    }
}
