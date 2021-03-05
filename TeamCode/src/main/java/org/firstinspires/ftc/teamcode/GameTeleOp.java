package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Game TeleOp")
public class GameTeleOp extends LinearOpMode {
    DcMotor rightFront, rightBack, leftFront, leftBack, shooter, intake, wobbleArm;
    Servo wobbleClaw, backPlate, flicker, wobbleLifter, ringKnocker;
    TouchSensor wobbleTouch1, wobbleTouch2;

    //Global Game State Variable
    GameState gameState = GameState.Intake;

    //Logic for RTM
    boolean gamePrevButtonState = false,
            gameCurrentButtonState = false;

    //Logic for Reversing Intake Direction
    IntakeDirection intakeDirection = IntakeDirection.In;
    boolean intakePrevButtonState = false,
            intakeCurrentButtonState = false;

    //Logic for keeping the intake on for a while
    boolean keepIntakeOn = false;
    double intakeStayOnTime = 0.5;
    double intakeStartTime,
            extraIntakeRunTime;

    //Logic for the Flicker
    double flickerStartTime,
            timeSinceFlicker;
    boolean firstReturn = true,
            tryFLick = false;

    //Logic for Shooter
    double shooterStartTime,
            shooterLastTime = 0,
            shooterLastRevolutions = 0,
            shooterRPM = 0,
            shooterRevolutionChange,
            shooterTimeChange,
            normalTargetRPM = 4200,
            powerShotTargetRPM = 3500,
            powerShotStartPower = .82,
            shooterTargetRPM = normalTargetRPM,
            shooterTotalRevolutions, shooterRunTime = 0,
            shooterStartPower = .85,
            shooterCurrentPower = shooterStartPower;

    //Logic for Power Shots
    ShooterState shooterState = ShooterState.Normal;
    boolean powerShotCurrentButton = false;
    boolean powerShotPrevButton = false;

    //Logic for Wobble Claw and Arm
    boolean currentClawButtonState, prevClawButtonState = false;
    double leftTrigger, rightTrigger, wobblePower;
    ClawState clawState = ClawState.Closed;


    public void runOpMode() throws InterruptedException {

        DefineHardwareMap();

        SetMotorDirectionAndMode();

        //Open Wobble Claw
        wobbleClaw.setPosition(1);

        backPlate.setPosition(1);
        flicker.setPosition(1);

        wobbleLifter.setPosition(0.45);
        ringKnocker.setPosition(0.5);

        waitForStart();

        shooterStartTime = System.nanoTime();
        flickerStartTime = System.nanoTime();

        SwitchToIntake();

        while(opModeIsActive()) {

            //Do all functions to run the motors at the right speeds
            MotorCode();

            //Claw and Wobble Arm Code
            WobbleSubroutine();

            //Shooting to Intake Subroutine
            ChangeGameStateSubroutine();

            if(gameState == GameState.Shooting) {
                Shooting();
            } else {
                Intake();
            }

            Flick();

            NormalToPowerShot();

            if(keepIntakeOn) {
                IntakeTimer();
            }
        }
    }

    void IntakeTimer() {
        extraIntakeRunTime = (System.nanoTime() - intakeStartTime) / TimeUnit.SECONDS.toNanos(1);

        if(extraIntakeRunTime > intakeStayOnTime) {
            intake.setPower(0);
            keepIntakeOn = false;
        }
    }

    void NormalToPowerShot() {
        powerShotCurrentButton = gamepad2.y;
        if(powerShotCurrentButton && powerShotCurrentButton != powerShotPrevButton) {
            if(shooterState == ShooterState.Normal) {
                shooterState = ShooterState.PowerShot;
                shooterCurrentPower = powerShotStartPower;
                shooterTargetRPM = powerShotTargetRPM;
            } else {
                shooterState = ShooterState.Normal;
                shooterCurrentPower = shooterStartPower;
                shooterTargetRPM = normalTargetRPM;
            }
        }
        telemetry.addData("shooter mode", shooterState);
        telemetry.update();
        powerShotPrevButton = powerShotCurrentButton;
    }

    void Intake() {
        intakeCurrentButtonState = gamepad2.b;

        if(intakeCurrentButtonState != intakePrevButtonState && intakeCurrentButtonState) {
            if (intakeDirection == IntakeDirection.In) {
                intake.setPower(-1);
                intakeDirection = IntakeDirection.Out;
            } else if (intakeDirection == IntakeDirection.Out) {
                intake.setPower(1);
                intakeDirection = IntakeDirection.In;
            }
        }
        intakePrevButtonState = intakeCurrentButtonState;
    }

    void Shooting() {
        double encoderPosition = shooter.getCurrentPosition();
        shooterTotalRevolutions = encoderPosition / 28;
        shooterRunTime = (System.nanoTime() - shooterStartTime) / TimeUnit.SECONDS.toNanos(1);

        if(shooterRunTime - shooterLastTime > 0.05) {
            //Get RPM

            shooterRevolutionChange = shooterTotalRevolutions - shooterLastRevolutions;
            shooterTimeChange = shooterRunTime - shooterLastTime;
            shooterRPM = (shooterRevolutionChange / shooterTimeChange) * 60;

            shooterLastRevolutions = shooterTotalRevolutions;
            shooterLastTime += 0.05;

            if(shooterRPM < shooterTargetRPM) {
                shooterCurrentPower += 0.005;
            } else if(shooterRPM > shooterTargetRPM) {
                shooterCurrentPower -= 0.005;
            }

            if(shooterCurrentPower > 1) {
                shooterCurrentPower = 1;
            } else if(shooterCurrentPower < 0 ) {
                shooterCurrentPower = 0;
            }

            shooter.setPower(shooterCurrentPower);
        }

        shooter.setPower(shooterCurrentPower);
    }

    void Flick() {
        timeSinceFlicker = (System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1);
        if(timeSinceFlicker >= 0.5) {
            if(firstReturn) {
                flicker.setPosition(1);
                firstReturn = false;
            }
        }
        tryFLick = gamepad2.left_bumper || gamepad2.right_bumper;
        if (timeSinceFlicker >= 1 && tryFLick) {
            flicker.setPosition(0);
            flickerStartTime = System.nanoTime();
            firstReturn = true;
        }
    }


    void ChangeGameStateSubroutine() {
        gameCurrentButtonState = gamepad2.a;

        if(gameCurrentButtonState != gamePrevButtonState && gameCurrentButtonState)
            if(gameState == GameState.Intake) {
                SwitchToShooting();
            }else {
                SwitchToIntake();
            }
        gamePrevButtonState = gameCurrentButtonState;
    }

    void SwitchToIntake() {
        backPlate.setPosition(1);

        gameState = GameState.Intake;

        intakeDirection = IntakeDirection.In;

        shooter.setPower(0);
        intake.setPower(1);
    }

    private void SwitchToShooting() {
        backPlate.setPosition(0);

        gameState = GameState.Shooting;

        intake.setPower(0);

        shooterStartTime = System.nanoTime();

        keepIntakeOn = true;
        intake.setPower(1);
        intakeStartTime = System.nanoTime();
    }

    private void WobbleSubroutine() {

        //Claw Toggle
        currentClawButtonState = gamepad2.x;
        if(currentClawButtonState && currentClawButtonState != prevClawButtonState) {
            if(clawState == ClawState.Open)
                clawState = ClawState.Closed;
            else
                clawState = ClawState.Open;
        }
        prevClawButtonState = currentClawButtonState;
        Claw();

        //Wobble Arm Code
        rightTrigger = gamepad2.right_trigger;
        leftTrigger = gamepad2.left_trigger;


        if(leftTrigger != 0 && rightTrigger != 0) {
            wobblePower = 0;
        }
        else if(leftTrigger!= 0) {
            wobblePower = 0.37;
        } else if (rightTrigger != 0) {
            wobblePower = -0.45;
        } else {
            wobblePower = 0;
        }

        if(wobbleTouch1.isPressed() && wobblePower < 0) {
            wobblePower = 0;
        } else if (wobbleTouch2.isPressed() && wobblePower > 0) {
            wobblePower = 0;
        }

        wobbleArm.setPower(wobblePower);
    }

    private void Claw() {
        if(clawState==ClawState.Open)
            wobbleClaw.setPosition(0);
        else
            wobbleClaw.setPosition(1);
    }

    private void DefineHardwareMap() {
        //For the drive wheels
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");

        //Shooter
        shooter = hardwareMap.dcMotor.get("shooter");

        //Intake
        intake = hardwareMap.dcMotor.get("intake");
        ringKnocker = hardwareMap.servo.get("ring_knocker");

        //For the Wobble Goal
        wobbleClaw = hardwareMap.servo.get("wobble_claw");
        wobbleArm = hardwareMap.dcMotor.get("wobble_arm");
        wobbleLifter = hardwareMap.servo.get("wobble_lifter");
        wobbleTouch1 = hardwareMap.touchSensor.get("wobble_touch1");
        wobbleTouch2 = hardwareMap.touchSensor.get("wobble_touch2");

        //For the Flicker and Backplate
        backPlate = hardwareMap.servo.get("backplate");
        flicker = hardwareMap.servo.get("flicker");
    }

    private void SetMotorDirectionAndMode() {
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            FrontRight *= 0.9;
            FrontLeft *= 0.9;
            BackLeft *= 0.9;
            BackRight *= 0.9;
        } else {
            FrontRight *= 0.55;
            FrontLeft *= 0.55;
            BackLeft *= 0.55;
            BackRight *= 0.55;
        }

        // write the values to the motors
        rightFront.setPower(FrontRight);
        leftFront.setPower(FrontLeft);
        leftBack.setPower(BackLeft);
        rightBack.setPower(BackRight);
    }

    private enum GameState {
        Shooting, Intake;
    }

    private enum IntakeDirection {
        In, Out;
    }

    private enum ShooterState {
        Normal, PowerShot;
    }

    private enum ClawState {
        Open, Closed;
    }
}