package org.firstinspires.ftc.teamcode.Config.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

@Config
public class Intake {
    public static double INTAKE_POWER = 1;
    public static double REVERSE_POWER = -1;
    public static double STOP_POWER = 0.0;
    public static double INTAKE_DOWN = 0;
    public static double INTAKE_UP = 1;
    private final ServoExGroup servoExGroup;
    private DcMotorEx motor;
    private ServoEx ArmLeft;
    private ServoEx ArmRight;
    public Intake(HardwareMap HwMap)
    {
        motor = HwMap.get(DcMotorEx.class, "intake");
        ArmLeft = HwMap.get(ServoEx.class , "ArmLeft");
        ArmRight = HwMap.get(ServoEx.class , "ArmRight");
        ArmLeft.setInverted(true);
        servoExGroup = new ServoExGroup(ArmLeft, ArmRight);
    }
    public void on()
    {
        motor.setPower(INTAKE_POWER);
        intakedown();
    }
    public void reverse()
    {
        motor.setPower(REVERSE_POWER);
        intakedown();
    }
    public void stop()
    {
        motor.setPower(STOP_POWER);
        setIntakeUp();
    }
    public void intakedown()
    {
        servoExGroup.set(INTAKE_DOWN);
    }
    public void setIntakeUp()
    {
        servoExGroup.set(INTAKE_UP);
    }
    /** IVY **/
    public CommandBuilder In() {
        return Commands.instant(this::on);
    }
    public CommandBuilder Out() {
        return Commands.instant(this::reverse);
    }
    public CommandBuilder Off() {
        return Commands.instant(this::stop);
    }
}
