package org.firstinspires.ftc.teamcode.OpModes.Test;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Motor test" , group = "Test")
public class TestMotor extends OpMode {
        public static String name = "";
        public static double power = 0;
        DcMotor motor;
        @Override
        public void init() {
            motor = hardwareMap.get(DcMotor.class, name);
        }
        @Override
        public void loop() {
            motor.setPower(power);
        }
}
