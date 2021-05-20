package org.firstinspires.ftc.teamcode.tools;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTuner extends LinearOpMode {
    public Servo servo;
    public static double position = 0.665;
    public double lastPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.servo.get("wobble_lifter");
        waitForStart();
        while(opModeIsActive()) {
            if (position != lastPos) {
                servo.setPosition(position);
            }
            lastPos = position;
        }
    }
}
