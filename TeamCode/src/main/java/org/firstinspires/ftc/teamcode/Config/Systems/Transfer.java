package org.firstinspires.ftc.teamcode.Config.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {
    public static double INTAKE_POWER = 1;
    public static double REVERSE_POWER = -1;
    public static double STOP_POWER = 0.0;
    private DcMotorEx motor;
    public Transfer(HardwareMap HwMap)
    {
        motor = HwMap.get(DcMotorEx.class, "transfer");
    }
    public void on()
    {
        motor.setPower(INTAKE_POWER);
    }
    public void reverse()
    {
        motor.setPower(REVERSE_POWER);
    }
    public void stop()
    {
        motor.setPower(STOP_POWER);
    }

    /** IVY **/
    public CommandBuilder DoTransfer() {
        return Commands.instant(this::on);
    }
    public CommandBuilder ReverseTransfer() {
        return Commands.instant(this::reverse);
    }
    public CommandBuilder Stop() {
        return Commands.instant(this::stop);
    }
}
