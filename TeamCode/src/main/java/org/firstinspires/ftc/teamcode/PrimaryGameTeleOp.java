package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry.Tools.GlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Odometry.Tools.MathFunctions;

import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

/**
 * Created by Jacob on 3-12-21
 */


/* NOTE
    If you ever need to read this without me here to tell you what it all means, just go through the code. You will see different
    things with names like gamepad1 and gamepad2. Those have button inputs with the game controllers associated with them.
    Run this OpMode and push all the buttons that you see to find out what they do. That should show you what most of the code does.
 */

@TeleOp(name="Game Teleop")
public class PrimaryGameTeleOp extends LinearOpMode {
    RevBlinkinLedDriver lights, lights2;
    DcMotor rightFront, rightBack, leftFront, leftBack, intake, wobbleArm;
    DcMotorEx shooter;
    DcMotor verticalLeft, verticalRight, horizontal;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, wobbleLifter, ringKnocker;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;

    //Global Game State Variable
    GameState gameState = GameState.Intake;
    DriveState driveState = DriveState.Normal;

    //Logic for Odometry
    boolean currentAState = false;
    boolean prevAState = false;

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
            normalTargetRPM = 4400,
            powerShotTargetRPM = 3800,
            shooterTargetRPM = normalTargetRPM;

    //Logic for Power Shots
    ShooterState shooterState = ShooterState.Normal;
    boolean powerShotCurrentButton = false;
    boolean powerShotPrevButton = false;

    //Logic for Wobble Claw and Arm
    boolean currentClawButtonState, prevClawButtonState = false;
    double leftTrigger, rightTrigger, wobblePower;
    ClawState clawState = ClawState.Closed;

    //Logic for the Wobble State Machine.
    //There will still be an optional button to open or close the claw manually, but it will be primarily automatic
    WobbleState wobbleState = WobbleState.Initial;
    double wobblePowerToUse = 0.75;
    boolean x, dpad_left, dpad_right;

    //Logic for putting the arm back at the beginning of the TeleOp
    boolean allowManualControl = false;
    boolean touchPressed = false;
    boolean currentWobbleMachineState, prevWobbleMachineState;
    int step = 1;
    double wobbleTimerStartTime = 0, timeSinceWobbleStart = 0;

    double distanceToTarget, robotX, robotY, robotOrientation, absoluteAngleToTarget, relativeAngleToTarget,
            relativeXToPoint, relativeYToPoint, movementXPower, movementYPower,
            movement_x, movement_y, movement_turn;
    double MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
    double leftFrontPower, rightFrontPower, rightBackPower, leftBackPower, lfAbs, rfAbs, rbAbs, lbAbs;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels.
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    GlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        //Initialize hardware map values.
        InitDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        DefineHardwareMap();

        SetMotorDirectionAndMode();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        //Open Wobble Claw
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);

        backPlate.setPosition(1);
        flicker.setPosition(1);

        wobbleLifter.setPosition(0.66);
        ringKnocker.setPosition(0);

        waitForStart();

        shooterStartTime = System.nanoTime();
        flickerStartTime = System.nanoTime();

        SwitchToIntake();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions.
        globalPositionUpdate = new GlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        waitForStart();

        wobbleArm.setPower(wobblePowerToUse);


        while(opModeIsActive()) {

            //Initial putting the wobble goal up thing
            if(!allowManualControl) {
                allowManualControl = wobbleTouch2.isPressed();
                if(allowManualControl) {
                    wobbleArm.setPower(0);
                    wobbleClaw.setPosition(1);
                    wobbleClaw2.setPosition(1);
                    wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    touchPressed = true;
                }
            }

            //Do all functions to run the motors at the right speeds
            DriveStateRoutine();

            //Claw and Wobble Arm Code
            //WobbleStateSubroutine();
            WobbleSubroutine();

            //Shooting to Intake Subroutine
            ChangeGameStateSubroutine();

            if(gameState == GameState.Shooting) {
                //Shooting();
                shooter.setVelocity((shooterTargetRPM*28)/60);
            } else {
                Intake();
            }

            Flick();

            NormalToPowerShot();

            if(keepIntakeOn) {
                IntakeTimer();
            }

            //For one player
            if(gamepad1.dpad_down) {
                ResetOdometry();
            }
            //Same for two players


            Telemetry();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    void DriveStateRoutine() {
        if(driveState == DriveState.Normal) {
            MotorCode();
        }else {
            if(distanceToTarget > 5) {
                stayAtAbsoluteAngle(0, 0, 1, 10, 0.5);
            }else if(distanceToTarget > 0.25){
                stayAtAbsoluteAngle(0, 0, 0.25, 0.25, 0.25);
            } else {
                driveState = DriveState.Normal;
            }
        }

        currentAState = gamepad1.a;

        if(currentAState && currentAState != prevAState) {
            if(driveState == DriveState.Normal) {
                driveState = DriveState.Odometry;
                SwitchToOdometry();
            } else if(driveState == DriveState.Odometry) {
                driveState = DriveState.Normal;
            }
        }

        prevAState = currentAState;
    }

    void SwitchToOdometry() {
        stayAtAbsoluteAngle(0, 0, 1, 5, 0.5);
    }

    void ResetOdometry() {
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        globalPositionUpdate.setZero();
    }

    void Telemetry() {
        telemetry.addData("wobble counts", wobbleArm.getCurrentPosition());
        telemetry.addData("wobble mode", wobbleState);
        telemetry.addData("step", step);
        telemetry.addData("touch 1", wobbleTouch1.isPressed());
        telemetry.addData("touch 2", wobbleTouch2.isPressed());
        telemetry.addData("Touch pressed", touchPressed);
        telemetry.addData("Allow manual control", allowManualControl);
        telemetry.addData("Left Trigger", leftTrigger);
        telemetry.addData("Right Trigger", rightTrigger);
        telemetry.addData("Wobble Power", wobblePower);
        telemetry.addData("shooter mode", shooterState);
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", MathFunctions.interpretAngle(globalPositionUpdate.returnOrientation()));
        telemetry.update();
    }

    void IntakeTimer() {
        extraIntakeRunTime = (System.nanoTime() - intakeStartTime) / TimeUnit.SECONDS.toNanos(1);

        if(extraIntakeRunTime > intakeStayOnTime) {
            intake.setPower(0);
            intakeServo.setPower(0);
            keepIntakeOn = false;
        }
    }

    void NormalToPowerShot() {
        //For one player
        //powerShotCurrentButton = gamepad1.y;
        //For two players
        powerShotCurrentButton = gamepad2.y;
        if(powerShotCurrentButton && powerShotCurrentButton != powerShotPrevButton) {
            if(shooterState == ShooterState.Normal) {
                shooterState = ShooterState.PowerShot;
                shooterTargetRPM = powerShotTargetRPM;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else {
                shooterState = ShooterState.Normal;
                shooterTargetRPM = normalTargetRPM;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }
        powerShotPrevButton = powerShotCurrentButton;
    }

    void Intake() {
        //For one player
        //intakeCurrentButtonState = gamepad1.b;
        //For two players
        intakeCurrentButtonState = gamepad2.b;

        if(intakeCurrentButtonState != intakePrevButtonState && intakeCurrentButtonState) {
            if (intakeDirection == IntakeDirection.In) {
                intake.setPower(-1);
                intakeDirection = IntakeDirection.Out;
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (intakeDirection == IntakeDirection.Out) {
                intake.setPower(1);
                intakeDirection = IntakeDirection.In;
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }
        intakePrevButtonState = intakeCurrentButtonState;
    }

    void Flick() {
        timeSinceFlicker = (System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1);
        if(timeSinceFlicker >= 0.5) {
            if(firstReturn) {
                flicker.setPosition(1);
                firstReturn = false;
            }
        }
        //For one player
        //tryFLick = gamepad1.left_trigger != 0;
        //For two players
        tryFLick = gamepad2.left_bumper || gamepad2.right_bumper;
        if (timeSinceFlicker >= 1 && tryFLick) {
            flicker.setPosition(0);
            flickerStartTime = System.nanoTime();
            firstReturn = true;
        }
    }

    void ChangeGameStateSubroutine() {
        //For one player
        //gameCurrentButtonState = gamepad1.a;
        //For two players
        gameCurrentButtonState = gamepad2.a;

        if(gameCurrentButtonState != gamePrevButtonState && gameCurrentButtonState)
            if(gameState == GameState.Intake) {
                SwitchToShooting();
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }else {
                SwitchToIntake();
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        gamePrevButtonState = gameCurrentButtonState;
    }

    void SwitchToIntake() {
        backPlate.setPosition(1);

        gameState = GameState.Intake;

        intakeDirection = IntakeDirection.In;

        shooter.setPower(0);
        intake.setPower(1);
        intakeServo.setPower(1);
    }

    private void SwitchToShooting() {
        backPlate.setPosition(0);

        gameState = GameState.Shooting;
        shooter.setVelocity((shooterTargetRPM*28)/60);

        intake.setPower(0);

        shooterStartTime = System.nanoTime();

        keepIntakeOn = true;
        intake.setPower(1);
        intakeStartTime = System.nanoTime();
    }

    private void WobbleStateSubroutine() {
        x = gamepad2.x;
        dpad_left = gamepad2.dpad_left;
        dpad_right = gamepad2.dpad_right;

        //For one player
        //currentWobbleMachineState = gamepad1.x;
        //For two players
        currentWobbleMachineState = x || dpad_left || dpad_right;

        if(currentWobbleMachineState && currentWobbleMachineState != prevWobbleMachineState) {
            if(x && !(dpad_left || dpad_right)) {
                if(wobbleState == WobbleState.Initial) {
                    allowManualControl = false;
                    wobbleState = WobbleState.DownOpen;
                    //wobbleArm.setPower(-wobblePowerToUse);
                    wobblePower = -wobblePowerToUse;
                    wobbleClaw.setPosition(0);
                    wobbleClaw2.setPosition(0);
                    step = 1;
                } else if (wobbleState == WobbleState.DownOpen) {
                    allowManualControl = false;
                    wobbleState = WobbleState.VerticalClosed;
                    step = 1;
                } else if (wobbleState == WobbleState.VerticalClosed) {
                    allowManualControl = false;
                    wobbleState = WobbleState.DropWobble;
                    //wobbleArm.setPower(-wobblePowerToUse);
                    wobblePower = -wobblePowerToUse;
                    step = 1;
                } else if (wobbleState == WobbleState.DropWobble) {
                    allowManualControl = false;
                    wobbleState = wobbleState.DownOpen;
                    //wobbleArm.setPower(-wobblePowerToUse);
                    wobblePower = -wobblePowerToUse;
                    wobbleClaw.setPosition(0);
                    wobbleClaw2.setPosition(0);
                    step = 1;
                } else if (wobbleState == WobbleState.RetractedClosed) {
                    allowManualControl = false;
                    wobbleState = WobbleState.DownOpen;
                    //wobbleArm.setPower(-wobblePowerToUse);
                    wobblePower = -wobblePowerToUse;
                    wobbleClaw.setPosition(0);
                    wobbleClaw2.setPosition(0);
                    step = 1;
                }
            } else if(dpad_left || dpad_right) {
                allowManualControl = false;
                wobbleState = WobbleState.RetractedClosed;
                wobbleClaw.setPosition(1);
                wobbleClaw2.setPosition(1);
                //wobbleArm.setPower(wobblePowerToUse);
                wobblePower = wobblePowerToUse;
                step = 1;
            }
        }

        prevWobbleMachineState = currentWobbleMachineState;

        if(wobbleState == WobbleState.Initial) {
            //Do nothing
        } else if (wobbleState == WobbleState.DownOpen) {
            if(step == 2) {
                allowManualControl = true;
            } else {
                wobblePower = -wobblePowerToUse;
                if (wobbleTouch1.isPressed()) {
                    //wobbleArm.setPower(0);
                    wobblePower = 0;
                    step = 2;
                }
            }
        } else if (wobbleState == WobbleState.VerticalClosed) {
            if(step==1) {
                wobbleClaw.setPosition(1);
                wobbleClaw2.setPosition(1);

                wobbleTimerStartTime = System.nanoTime();

                step = 2;
            } else if (step==2) {
                timeSinceWobbleStart = (System.nanoTime() - wobbleTimerStartTime) / TimeUnit.SECONDS.toNanos(1);
                if(timeSinceWobbleStart >= 0.5) {
                    //wobbleArm.setPower(wobblePowerToUse);
                    wobblePower = wobblePowerToUse;
                    step = 3;
                }
            }else if (step == 4) {
                allowManualControl = true;
            } else if (step==3) {
                if(wobbleArm.getCurrentPosition() >= -800) {
                    //wobbleArm.setPower(0);
                    wobblePower = 0;
                    step = 4;
                }
            }
        } else if (wobbleState == WobbleState.DropWobble) {
            if(step ==1) {
                if(wobbleArm.getCurrentPosition() <= -1300) {
                    //wobbleArm.setPower(0);
                    wobblePower = 0;

                    wobbleClaw.setPosition(0);
                    wobbleClaw2.setPosition(0);
                    wobbleTimerStartTime = System.nanoTime();

                    step = 2;
                }
            } else if (step == 2) {
                timeSinceWobbleStart = (System.nanoTime() - wobbleTimerStartTime) / TimeUnit.SECONDS.toNanos(1);
                if(timeSinceWobbleStart >= 1) {
                    allowManualControl = true;
                }
            }

        } else if (wobbleState == WobbleState.RetractedClosed) {
            if(step ==2) {
                allowManualControl = true;
            } else if(wobbleTouch2.isPressed()) {
                //wobbleArm.setPower(0);
                wobblePower = 0;
                step = 2;
            }
        }
    }

    private void WobbleSubroutine() {
        //Claw Toggle
        //For one player
        //currentClawButtonState = gamepad1.right_trigger != 0;
        // two players
        currentClawButtonState = gamepad2.x;
        if(currentClawButtonState && currentClawButtonState != prevClawButtonState) {
            if(clawState == ClawState.Open) {
                clawState = ClawState.Closed;
                wobbleClaw.setPosition(1);
                wobbleClaw2.setPosition(1);
            }else {
                clawState = ClawState.Open;
                wobbleClaw.setPosition(0);
                wobbleClaw2.setPosition(0);
            }
        }
        prevClawButtonState = currentClawButtonState;


        //Uncomment all of this to give manual wobble arm control back.

        //Wobble Arm Code
        //For one player
        //rightTrigger = gamepad1.right_trigger;
        //For two players
        rightTrigger = gamepad2.right_trigger;
        //For one player
        //leftTrigger = gamepad1.left_trigger;
        //For two players
        leftTrigger = gamepad2.left_trigger;


        if(leftTrigger != 0 && rightTrigger != 0 && allowManualControl) {
            wobblePower = 0;
        } else if(leftTrigger!= 0 && allowManualControl) {
            wobblePower = wobblePowerToUse;
        } else if (rightTrigger != 0 && allowManualControl) {
            wobblePower = -wobblePowerToUse;
        } else if(rightTrigger == 0 && leftTrigger == 0 && allowManualControl){
            wobblePower = 0;
        }

        if(wobbleTouch1.isPressed() && wobblePower < 0 && allowManualControl) {
            wobblePower = 0;
        } else if (wobbleTouch2.isPressed() && wobblePower > 0 && allowManualControl) {
            wobblePower = 0;
            wobbleClaw.setPosition(1);
            wobbleClaw2.setPosition(1);
        }
        if(touchPressed) {
            wobbleArm.setPower(wobblePower);
        }
    }

    private void DefineHardwareMap() {
        //For the drive wheels
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");

        //Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        //Intake
        intake = hardwareMap.dcMotor.get("intake");
        ringKnocker = hardwareMap.servo.get("ring_knocker");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        //For the Wobble Goal
        wobbleClaw = hardwareMap.servo.get("wobble_claw");
        wobbleClaw2 = hardwareMap.servo.get("wobble_claw2");
        wobbleArm = hardwareMap.dcMotor.get("wobble_arm");
        wobbleLifter = hardwareMap.servo.get("wobble_lifter");
        wobbleTouch1 = hardwareMap.touchSensor.get("wobble_touch1");
        wobbleTouch2 = hardwareMap.touchSensor.get("wobble_touch2");

        //For the Flicker and Backplate
        backPlate = hardwareMap.servo.get("backplate");
        flicker = hardwareMap.servo.get("flicker");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights2 = hardwareMap.get(RevBlinkinLedDriver.class, "lights2");

    }

    private void SetMotorDirectionAndMode() {
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void stayAtAbsoluteAngle(double x, double y, double movementSpeed, double error, double turnSpeed) {
        distanceToTarget = Math.hypot(x - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH), y - (-globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH));

        robotX = globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
        robotY = -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;

        if(distanceToTarget > error) {

            robotX = globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x - (robotX), y - (robotY));

            absoluteAngleToTarget = Math.atan2(y - robotY, x - robotX);
            relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
            movementXPower = relativeXToPoint / MovementPowerDenominator;
            movementYPower = relativeYToPoint / MovementPowerDenominator;

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            double turnToAbsoluteAngleToTarget = Math.atan2((robotY+10) - robotY, robotX - robotX);
            double turnToRelativeAngleToTarget = MathFunctions.AngleWrap(turnToAbsoluteAngleToTarget - (robotOrientation));

            double turnToRelativeTurnAngle = turnToRelativeAngleToTarget - toRadians(90);
            movement_turn = clip(turnToRelativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;

            if (distanceToTarget < 5) {
                movement_turn = 0;
            }

            CalculateMotorPowers(movement_x, movement_y, movement_turn);
        }
    }

    //Function to initialize all the motors and encoders. Used when preparing for the OpMode to start.
    void InitDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        rightFront = hardwareMap.dcMotor.get(rfName);
        rightBack = hardwareMap.dcMotor.get(rbName);
        leftFront = hardwareMap.dcMotor.get(lfName);
        leftBack = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private enum GameState {
        Shooting, Intake
    }

    private enum IntakeDirection {
        In, Out
    }

    private enum ShooterState {
        Normal, PowerShot
    }

    private enum ClawState {
        Open, Closed
    }

    private enum DriveState {
        Normal, Odometry
    }

    private enum WobbleState {
        Initial, DownOpen, VerticalClosed, DropWobble, RetractedClosed
    }

    public double interpretAngle(double Orientation) {
        if(isWithin(Orientation, -180, 0)) {
            return Orientation;
        } else if (isWithin(Orientation, -360, -180)) {
            return (Orientation +360);
        } else {
            return Orientation;
        }
    }

    public boolean isWithin(double value, double min, double max) {
        if(value >= max) {
            return false;
        } else if (value <= min) {
            return false;
        } else {
            return true;
        }
    }

    private void CalculateMotorPowers(double movement_x, double movement_y, double movement_turn) {
        leftFrontPower = -movement_y - movement_x + movement_turn;
        rightFrontPower = movement_y - movement_x + movement_turn;
        rightBackPower = movement_y + movement_x + movement_turn;
        leftBackPower = -movement_y + movement_x + movement_turn;

        //Scale Motor Powers to preserve movement shape
        lfAbs = abs(leftFrontPower);
        rfAbs = abs(rightFrontPower);
        rbAbs = abs(rightBackPower);
        lbAbs = abs(leftBackPower);

        if(lfAbs > rfAbs && lfAbs > rbAbs && lfAbs > lbAbs && lfAbs > 1) {
            leftFrontPower = leftFrontPower/lfAbs;
            rightFrontPower = rightFrontPower/lfAbs;
            rightBackPower = rightBackPower/lfAbs;
            leftBackPower = leftBackPower/lfAbs;
        } else if(rfAbs > lfAbs && rfAbs > rbAbs && rfAbs > lbAbs && rfAbs > 1) {
            leftFrontPower = leftFrontPower/rfAbs;
            rightFrontPower = rightFrontPower/rfAbs;
            rightBackPower = rightBackPower/rfAbs;
            leftBackPower = leftBackPower/rfAbs;
        } else if(rbAbs > lfAbs && rbAbs > rfAbs && rbAbs > lbAbs && rbAbs > 1) {
            leftFrontPower = leftFrontPower/rbAbs;
            rightFrontPower = rightFrontPower/rbAbs;
            rightBackPower = rightBackPower/rbAbs;
            leftBackPower = leftBackPower/rbAbs;
        } else if(lbAbs > lfAbs && lbAbs > rfAbs && lbAbs > rbAbs && lbAbs > 1) {
            leftFrontPower = leftFrontPower/lbAbs;
            rightFrontPower = rightFrontPower/lbAbs;
            rightBackPower = rightBackPower/lbAbs;
            leftBackPower = leftBackPower/lbAbs;
        }

        leftFrontPower = clip(leftFrontPower, -1, 1);
        rightFrontPower = clip(rightFrontPower, -1, 1);
        rightBackPower = clip(rightBackPower, -1, 1);
        leftBackPower = clip(leftBackPower, -1, 1);

        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
    }
}
