package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Closed Loop Control Test")
public class closedLoopControl extends LinearOpMode {
    public DcMotor test;

    public void runOpMode() throws InterruptedException {
        double targetRPM = 6000;
        int totalRevolutions;
        double runTime = 0;
        double currentPower = 1;

        test = hardwareMap.dcMotor.get("test");

        test.setDirection(DcMotorSimple.Direction.REVERSE);

        test.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        test.setPower(currentPower);
        totalRevolutions = 0;

        double startTime = System.nanoTime();
        while(opModeIsActive()) {

            runTime = System.nanoTime() - startTime;

            getRuntime();

            telemetry.addData("Power", currentPower);
            telemetry.addData("Encoder Position", test.getCurrentPosition());
            telemetry.addData("Total Revolutions", totalRevolutions);
            telemetry.addData("Runtime (Nano)", runTime);
            telemetry.addData("Runtime (Sec)", runTime / TimeUnit.SECONDS.toNanos(1));
            telemetry.update();

        }
    }

}
