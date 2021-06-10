package org.firstinspires.ftc.teamcode.christiancode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class practice extends LinearOpMode {

    DcMotor rightFront,leftFront,rightBack,leftBack;
    @Override
    public void runOpMode() throws InterruptedException {
        rightFront=hardwareMap.get(DcMotor.class,"right_front");
        leftFront=hardwareMap.get(DcMotor.class,"left_front");
        rightBack=hardwareMap.get(DcMotor.class,"right_back");
        leftBack=hardwareMap.get(DcMotor.class,"left_back");

        waitForStart();
        while(opModeIsActive()) {
            //Get Gamepad values
            double gamepad1LeftY = gamepad1.left_stick_y*0.5;
            double gamepad1LeftX = -gamepad1.left_stick_x*.5;
            double gamepad1RightX = -gamepad1.right_stick_x*0.5;
            // holonomic formulas
            double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);
            leftBack.setPower(BackLeft);
            rightBack.setPower(BackRight);
            leftFront.setPower(FrontLeft);
            rightFront.setPower(FrontRight);

            telemetry.addData("power of motor rightFront",FrontRight);
            telemetry.update();
        }
    }


}
