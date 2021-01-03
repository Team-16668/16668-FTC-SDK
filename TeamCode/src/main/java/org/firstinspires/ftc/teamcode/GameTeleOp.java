package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Game TeleOp")
public class GameTeleOp extends LinearOpMode {
    DcMotor right_front, right_back, left_front, left_back, shooter, intake;
    Servo claw, wobbleArm, backPlateLeft, backPlateRight;

    //Global Game State Variable
    GameState state = GameState.Intake;

    //Logic for Wobble Claw and Arm
    boolean switchClaw = false;
    boolean switchArm = false;
    boolean prevStateClaw = false;
    boolean prevStateArm = false;
    double clawPos = 1;
    double armPos = 1;

    //Logic for RTM
    boolean gamePrevButtonState = false;
    boolean gameCurrentButtonState = false;

    //Logic for Reversing Intake Direction
    Direction direction = Direction.In;
    boolean intakePrevButtonState = false;
    boolean intakeCurrentButtonState = false;

    //Logic for Shooter
    double startTime;
    double shooterLastTime = 0;
    double shooterLastRevolutions = 0;
    double shooterRPM = 0;
    double shooterRevolutionChange, shooterTimeChange;

    double shooterTargetRPM = 3750;
    double shooterTotalRevolutions;
    double shooterRunTime = 0;
    double shooterCurrentPower = 0.85;


    public void runOpMode() throws InterruptedException {

        DefineHardwareMap();

        SetMotorDirectionAndMode();

        claw.setPosition(1);
        wobbleArm.setPosition(1);

        backPlateLeft.setPosition(0);
        backPlateRight.setPosition(1);

        waitForStart();

        startTime = System.nanoTime();

        while(opModeIsActive()) {

            //Do all functions to run the motors at the right speeds
            MotorCode();

            //Claw and Wobble Arm Code
            WobbleSubroutine();

            //Shooting to Intake Subroutine
            ChangeGameStateSubroutine();

            if(state == GameState.Shooting) {
                Shooting();
            } else {
                Intake();
            }

            telemetry.update();
        }
    }

    void Intake() {
        intakeCurrentButtonState = gamepad2.b;

        if(intakeCurrentButtonState != intakePrevButtonState && intakeCurrentButtonState) {
            if (direction == Direction.In) {
                intake.setPower(1);
            } else if (direction == Direction.Out) {
                intake.setPower(-1);
            }
        }
        gamePrevButtonState = gameCurrentButtonState;
    }

    void Shooting() {
        double encoderPosition = shooter.getCurrentPosition();
        shooterTotalRevolutions = encoderPosition / 28;
        shooterRunTime = (System.nanoTime() - startTime) / TimeUnit.SECONDS.toNanos(1);

        if(shooterRunTime - shooterLastTime > 0.05) {

            //Get RPM

            shooterRevolutionChange = shooterTotalRevolutions - shooterLastRevolutions;
            shooterTimeChange = shooterRunTime - shooterLastTime;
            shooterRPM = (shooterRevolutionChange / shooterTimeChange) * 60;

            shooterLastRevolutions = shooterTotalRevolutions;
            shooterLastTime += 0.05;

            if(shooterRPM < shooterTargetRPM) {
                shooterCurrentPower += 0.001;
            } else if(shooterRPM > shooterTargetRPM) {
                shooterCurrentPower -= 0.001;
            }

            if(shooterCurrentPower > 1) {
                shooterCurrentPower = 1;
            } else if(shooterCurrentPower < 0 ) {
                shooterCurrentPower = 0;
            }
        }

        shooter.setPower(shooterCurrentPower);

        telemetry.addData("Power", shooterCurrentPower);
        telemetry.addData("Revolutions", shooterTotalRevolutions);
        telemetry.addData("RPM", shooterRPM);
    }

    void ChangeGameStateSubroutine() {
        gameCurrentButtonState = gamepad2.a;

        if(gameCurrentButtonState != gamePrevButtonState && gameCurrentButtonState) {
            if (state == GameState.Intake) {
                SwitchToShooting();
            } else if (state == GameState.Shooting) {
                SwitchToIntake();
            }
        }
        gamePrevButtonState = gameCurrentButtonState;
    }

    void SwitchToIntake() {
        backPlateLeft.setPosition(0);
        backPlateRight.setPosition(1);

        state = GameState.Intake;

        shooter.setPower(0);

        telemetry.addData("State", "Intake");
    }

    void SwitchToShooting() {
        backPlateLeft.setPosition(1);
        backPlateRight.setPosition(0);

        state = GameState.Shooting;

        intake.setPower(0);

        startTime = System.nanoTime();

        telemetry.addData("State", "Shooting");
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

        //Shooter
        shooter = hardwareMap.dcMotor.get("shooter");

        //Intake
        intake = hardwareMap.dcMotor.get("intake");

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

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    private enum GameState {
        Shooting,
        Intake
    }

    private enum Direction {
        In,
        Out,
    }
}
