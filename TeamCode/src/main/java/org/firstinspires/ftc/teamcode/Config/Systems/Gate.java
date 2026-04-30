package org.firstinspires.ftc.teamcode.Config.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.Provider;

@Config // acmeorobotics dashboard config
public class Gate {
    private Servo servo;
    public static double OPEN = 0.81;
    public static double CLOSE = 0.6;
    public Gate(HardwareMap HwMap)
    {
        servo = HwMap.get(Servo.class, "gate");
    }
    public void open()
    {
        servo.setPosition(OPEN);
    }
    public void close()
    {
        servo.setPosition(CLOSE);
    }
}
