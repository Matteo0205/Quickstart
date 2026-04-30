package org.firstinspires.ftc.teamcode.Config.Util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;

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
                if(shooter.atTarget())
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
}
