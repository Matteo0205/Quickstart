package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TurretTicksCalibration", group = "Test")
public class TurretTicksCalibration extends LinearOpMode {

    private DcMotor encoderMotor;

    @Override
    public void runOpMode() {

        encoderMotor = hardwareMap.get(DcMotor.class, "encoderMotor");

        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int startTicks = 0;
        int currentTicks;

        boolean measuring = false;

        while (opModeIsActive()) {

            if (gamepad1.a && !measuring) {
                encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                startTicks = 0;
                measuring = true;
            }

            if (gamepad1.b && measuring) {
                measuring = false;
            }

            currentTicks = encoderMotor.getCurrentPosition();

            telemetry.addData("Ticks", currentTicks);
            telemetry.addData("Measuring", measuring);
            telemetry.addData("Sebi e" , "EPSTEIN");
            telemetry.update();
        }
    }
}