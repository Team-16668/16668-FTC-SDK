package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Game TeleOp")
public class GameTeleOp extends LinearOpMode {
    DcMotor right_front, right_back, left_front, left_back;
    Servo claw, wobbleArm, backPlateLeft, backPlateRight;

    //Logic for Wobble Claw and Arm
    boolean switchClaw = false;
    boolean switchArm = false;
    boolean prevStateClaw = false;
    boolean prevStateArm = false;
    double clawPos = 1;
    double armPos = 1;

    //Logic for RTM
    boolean prevButtonIntakeState = false;
    boolean currentButtonIntakeState = false;
    int intakeState = 0;

    public void runOpMode() throws InterruptedException {

        DefineHardwareMap();

        SetMotorDirectionAndMode();

        claw.setPosition(1);
        wobbleArm.setPosition(1);

        backPlateLeft.setPosition(0);
        backPlateRight.setPosition(1);

        waitForStart();

        while(opModeIsActive()) {

            //Do all functions to run the motors at the right speeds
            MotorCode();

            //Claw and Wobble Arm Code
            WobbleSubroutine();

            //Shooting to Intake Subroutine
            ShootToIntakeSubroutine();
        }
    }

    void ShootToIntakeSubroutine() {
        currentButtonIntakeState = gamepad2.a;

        if(currentButtonIntakeState != prevButtonIntakeState && currentButtonIntakeState) {
            if (intakeState == 0) {
                intakeState = 1;
                SwitchToShooting();
            } else if (intakeState == 1) {
                intakeState = 0;
                SwitchToIntake();
            }
        }
        prevButtonIntakeState = currentButtonIntakeState;
    }

    void SwitchToIntake() {
        backPlateLeft.setPosition(0);
        backPlateRight.setPosition(1);

        telemetry.addData("State", "Intake");
        telemetry.update();
    }

    void SwitchToShooting() {
        backPlateLeft.setPosition(1);
        backPlateRight.setPosition(0);

        telemetry.addData("State", "Shooting");
        telemetry.update();
    }

    void WobbleSubroutine() {
        CheckClaw();
        CheckWobbleArm();
    }

    void CheckClaw() {
        //Wobble Claw Code
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
    }

    void CheckWobbleArm() {
        //Wobble Arm Code
        if(gamepad1.b) {
            if(!prevStateArm) {
                switchArm = true;
            }
            prevStateArm = true;
        } else
            prevStateArm = false;

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

    private void DefineHardwareMap() {
        //For the drive wheels
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        //For the Wobble Goal
        claw = hardwareMap.servo.get("claw");
        wobbleArm = hardwareMap.servo.get("wobble_arm");
        backPlateLeft = hardwareMap.servo.get("backplate_left");
        backPlateRight = hardwareMap.servo.get("backplate_right");
    }

    private void SetMotorDirectionAndMode() {
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void MotorCode() {
        //Get Gamepad values
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

        //Display information in Telemetry
        /*
        telemetry.addData(" lf", FrontLeft);
        telemetry.addData(" rf", FrontRight);
        telemetry.addData(" rb", BackRight);
        telemetry.addData(" lb", BackLeft);
        telemetry.update();
         */

        //Apply Speed Modifiers
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
    }
}
