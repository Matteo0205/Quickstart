package org.firstinspires.ftc.teamcode.OpModes.Test;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends OpMode {
    Servo servo;
    public static String name = "";
    public static double position = 0;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, name);
    }
    public void loop() {
        servo.setPosition(position);
    }
}
