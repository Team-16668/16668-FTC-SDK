package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Practice TeleOp")
public class practiceTeleOp extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;

    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("front_right");
        right_back = hardwareMap.dcMotor.get("back_right");
        left_front = hardwareMap.dcMotor.get("front_left");
        left_back = hardwareMap.dcMotor.get("back_left");

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive()) {

            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // holonomic formulas

            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            telemetry.addData(" lf", FrontLeft);
            telemetry.addData(" rf", FrontRight);
            telemetry.addData(" rb", BackRight);
            telemetry.addData(" lb", BackLeft);
            telemetry.update();

            if(gamepad1.left_bumper) {
                FrontRight *= 0.25;
                FrontLeft *= 0.25;
                BackLeft *= 0.25;
                BackRight *= 0.25;
            } else if(gamepad1.right_bumper) {
                FrontRight *= 0.55;
                FrontLeft *= 0.55;
                BackLeft *= 0.55;
                BackRight *= 0.55;
            }  else{
                FrontRight *= 0.35;
                FrontLeft *= 0.35;
                BackLeft *= 0.35;
                BackRight *= 0.35;
            }

            // write the values to the motors
            right_front.setPower(FrontRight);
            left_front.setPower(FrontLeft);
            left_back.setPower(BackLeft);
            right_back.setPower(BackRight);
        }
    }
}
