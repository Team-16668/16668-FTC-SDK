package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Closed Loop Control Test")
public class closedLoopControl extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;

    public void runOpMode() throws InterruptedException {
        double targetRPM = 6000;
        int totalRevolutions;
        double runTime = 0;
        double currentPower = 0.15;

        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        right_front.setPower(currentPower);
        totalRevolutions = 0;

        double startTime = System.nanoTime();
        while(opModeIsActive()) {

            runTime = System.nanoTime() - startTime;

            getRuntime();

            telemetry.addData("Power", currentPower);
            telemetry.addData("Encoder Position", right_back.getCurrentPosition());
            telemetry.addData("Total Revolutions", totalRevolutions);
            telemetry.addData("Runtime (Nano)", runTime);
            telemetry.addData("Runtime (Sec)", runTime / TimeUnit.SECONDS.toNanos(1));
            telemetry.update();

        }
    }

}
