package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Game TeleOp")
public class GameTeleOp extends LinearOpMode {
    public DcMotor right_front, right_back, left_front, left_back;
    public Servo claw, wobbleArm;

    //Logic for Wobble Claw and Arm
    public boolean switchClaw = false;
    public boolean switchArm = false;
    public boolean prevStateClaw = false;
    public boolean prevStateArm = false;

    public double clawPos = 1;
    public double armPos = 1;

    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        claw = hardwareMap.servo.get("claw");
        wobbleArm = hardwareMap.servo.get("wobble_arm");

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(1);
        wobbleArm.setPosition(1);

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
            }

            // write the values to the motors
            right_front.setPower(FrontRight);
            left_front.setPower(FrontLeft);
            left_back.setPower(BackLeft);
            right_back.setPower(BackRight);

            //Claw and Wobble Arm Code

            if(gamepad1.a) {
                if(!prevStateClaw) {
                    switchClaw = true;
                }
                prevStateClaw = true;
            } else {
                prevStateClaw = false;
            }

            if(switchClaw) {
                if(clawPos == 1) {
                    clawPos = 0;
                } else {
                    clawPos = 1;
                }
                switchClaw = false;
            }

            claw.setPosition(clawPos);

            if(gamepad1.b) {
                if(!prevStateArm) {
                    switchArm = true;
                }
                prevStateArm = true;
            } else {
                prevStateArm = false;
            }

            if(switchArm) {
                if(armPos == 1) {
                    armPos = 0;
                } else {
                    armPos = 1;
                }
                switchArm = false;
            }

            wobbleArm.setPosition(armPos);
        }
    }
}
