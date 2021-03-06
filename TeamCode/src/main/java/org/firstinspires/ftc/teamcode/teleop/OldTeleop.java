package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.util.RoadrunnerPoint;
import org.firstinspires.ftc.teamcode.roadrunner.util.TelemetryPacket;
import org.firstinspires.ftc.teamcode.vision.CustomPowershotPipelineRed;
import org.firstinspires.ftc.teamcode.vision.UGAngleHighGoalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@Disabled
@TeleOp(name= "Old Teleop")
public class OldTeleop extends LinearOpMode {
    //Initial Variable initialization
    //Hardware devices
    RevBlinkinLedDriver lights, lights2;
    DcMotor intake, wobbleArm;
    DcMotorEx shooter;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, wobbleLifter, ringKnocker, herderServoLeft, herderServoRight;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;
    private VoltageSensor batteryVoltageSensor;

    boolean allowPowershotMachine = false;
    int powerShotStep = 1;

    //All the State Machines
    GameState gameState = GameState.Intake;
    DriveState driveState = DriveState.DRIVER_CONTROL;
    Powershot currentPowerShot = Powershot.Right;
    IntakeDirection intakeDirection = IntakeDirection.In;
    ShooterState shooterState = ShooterState.Normal;
    ClawState clawState = ClawState.Closed;
    WobbleState wobbleState = WobbleState.Initial;
    HerderState herderState = HerderState.Up;
    AutomaticState automaticState = AutomaticState.GoToLine;

    boolean tryHerderDown = true;

    double wobbleLifterUpPos = 0.649;

    public static double leftFactor = 9;
    public static double rightFactor = 0;

    public static double leftShotOffset  = -3;
    public static double centerShotOffset = -3;
    public static double rightShotOffset = -4;

    //Global Game State Variable
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double goalXPos = 72;
    double goalYPos = -39;
    double rightGoalXPos = 72;
    double rightGoalYPos = -37;
    double currentXPos = goalXPos;
    double currentYPos = goalYPos;
    boolean currentBState = false;
    boolean prevBState = false;
    boolean dpadLeft;
    boolean dpadRight;
    boolean dpadCurrentSate;
    boolean dpadPrevState;
    RoadrunnerPoint powerShotShootingPointLeft = new RoadrunnerPoint(-10, -11);
    RoadrunnerPoint powerShotShootingPointCenter = new RoadrunnerPoint(-10, -19);
    RoadrunnerPoint powerShotShootingPointRight = new RoadrunnerPoint(-10, -27);
    RoadrunnerPoint currentPowerShotTargetPoint = new RoadrunnerPoint(powerShotShootingPointLeft.x, powerShotShootingPointLeft.y);

    public static double constraintRight = -48;
    public static double constraintLeft = -24;
    public static double constraintFront = -10;
    public static double constraintBack = -24;


    //Logic for Odometry
    boolean currentAState = false;
    boolean prevAState = false;
    double powerMultiplier;

    //Logic for RTM
    boolean gamePrevButtonState = false,
            gameCurrentButtonState = false;

    //Logic for Reversing Intake Direction
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

    public static double totalFlickTime = 0.5;

    //Logic for Shooter
    public static double normalTargetRPM = 3710;
    public static double powerShotTargetRPM = 3365;
    double shooterStartTime,
            shooterTargetRPM = normalTargetRPM;
    InterpLUT shooterLut;

    //Logic for Power Shots
    boolean powerShotCurrentButton = false;
    boolean powerShotPrevButton = false;

    //Logic for Wobble Claw and Arm
    boolean currentClawButtonState, prevClawButtonState = false;
    double leftTrigger, rightTrigger, wobblePower;

    //Logic for the Wobble State Machine.
    //There will still be an optional button to open or close the claw manually, but it will be primarily automatic
    double wobblePowerToUse = 0.75;
    boolean x, dpad_left, dpad_right;

    //Logic for putting the arm back at the beginning of the TeleOp
    boolean allowManualControl = false;
    boolean touchPressed = false;
    boolean currentWobbleMachineState, prevWobbleMachineState;
    int step = 1;
    double wobbleTimerStartTime = 0, timeSinceWobbleStart = 0;

    //Logic for the ring herder
    boolean currentDPad = false;
    boolean prevDPad = false;

    int turnStep = 1;

    SampleMecanumDriveCancelable drive;

    //Vision stuff
    OpenCvWebcam webcam;
    UGAngleHighGoalPipeline highGoalPipeline;
    CustomPowershotPipelineRed powershotPipeline;

    boolean currentYState, prevYState;
    boolean shouldFollow = true;

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "goal_webcam"), cameraMonitorViewId);
        highGoalPipeline = new UGAngleHighGoalPipeline(60, 0, 0);
        powershotPipeline = new CustomPowershotPipelineRed(60, 0, 0);

        webcam.setPipeline(highGoalPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        dashboard.startCameraStream(webcam, 0);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        //Initialize hardware map values.
        InitializeHardwaremap();

        double distanceToShooter = 60;

        //Initialize the dashboard
        dashboard = FtcDashboard.getInstance();
        shooterLut = new InterpLUT();
        shooterLut.add(0,4400);
        shooterLut.add(0+distanceToShooter,4400);
        shooterLut.add(8+distanceToShooter,4400);
        shooterLut.add(12+distanceToShooter,4400);
        shooterLut.add(16+distanceToShooter,4300);
        shooterLut.add(20+distanceToShooter,4200);
        shooterLut.add(24+distanceToShooter,4100);
        shooterLut.add(28+distanceToShooter,4100);
        shooterLut.add(32+distanceToShooter,4150);
        shooterLut.add(36+distanceToShooter,4200);
        shooterLut.add(40+distanceToShooter,4250);
        shooterLut.add(46+distanceToShooter,4300);
        shooterLut.add(52+distanceToShooter,4250);
        shooterLut.add(58+distanceToShooter,4350);
        //This one might need to be changed if the distance from far away is completely wack.
        shooterLut.add(500,4350);
        shooterLut.createLUT();

        //Set the Localizer/Positioning System
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                190, 0, 10, 13 /* * 12 / batteryVoltageSensor.getVoltage() */
        ));

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        //drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));
        drive.setPoseEstimate(PoseStorage.currentPose);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Put the wobble arm down by default
        wobbleArm.setPower(wobblePowerToUse);
        intake.setPower(1);
        intakeServo.setPower(1);
        shooterStartTime = System.nanoTime();
        flickerStartTime = System.nanoTime();

        while(opModeIsActive()) {

            drive.drawBoundingBox(constraintLeft, constraintRight, constraintBack, constraintFront);

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
                //This is the simple code for being on a square to the goal.
                Shooting();

            }

            Intake();

            Flick();

            NormalToPowerShot();

            if(keepIntakeOn) {
                IntakeTimer();
            }

            HerderSubroutine();

            Telemetry();
        }
        webcam.stopStreaming();
    }

    private void HerderSubroutine() {
        currentDPad = gamepad1.dpad_up || gamepad1.dpad_down;

        if(currentDPad && currentDPad != prevDPad) {
            if(herderState == HerderState.Up) {
                putHerdersDown();
            } else {
                putHerdersUp();
            }
        }
        prevDPad = currentDPad;

        Pose2d currentPose = drive.getPoseEstimate();
        if(currentPose.getX() > 36 || currentPose.getX() < -36 || currentPose.getY() > -12 || currentPose.getY() < -36) {
            putHerdersUp();
        } else if (tryHerderDown) {
            putHerdersDown();
            tryHerderDown = false;
        }
    }

    private void putHerdersDown() {
        herderServoLeft.setPosition(1);
        herderServoRight.setPosition(0);
        herderState = HerderState.Down;
    }

    private void putHerdersUp() {
        herderServoLeft.setPosition(0);
        herderServoRight.setPosition(1);
        herderState = HerderState.Up;
    }

    private void Shooting() {
        if(shooterState == ShooterState.Normal) {
            /*
            Pose2d myPose = drive.getPoseEstimate();
            shooterTargetRPM = drive.distanceFromPoint(myPose.getX(), myPose.getY(), goalXPos, goalYPos);
            double shooterVelocity = shooterLut.get(drive.distanceFromPoint(myPose.getX(), myPose.getY(), goalXPos, goalYPos));
            */
            //shooter.setVelocity((shooterVelocity*28)/60);
            shooter.setVelocity(normalTargetRPM*28/60);
            shooterTargetRPM = normalTargetRPM;
        } else if(shooterState == ShooterState.PowerShot) {
            shooterTargetRPM = powerShotTargetRPM;
            shooter.setVelocity((powerShotTargetRPM*28)/60);
        }
    }

    void Telemetry() {
        drive.telemetryPackets = new TelemetryPacket[]{
        new TelemetryPacket("velocity", Double.toString((shooter.getVelocity() / 28) * 60)),
        new TelemetryPacket("target velocity", Double.toString(shooterTargetRPM)),
        new TelemetryPacket("shooter mode", shooterState.name()),
        new TelemetryPacket("Drive mode", driveState.name()),
        new TelemetryPacket("Powershot", currentPowerShot.name()),
        new TelemetryPacket("Goal visisble", Boolean.toString(highGoalPipeline.isRedVisible())),
        new TelemetryPacket("Goal pitch", Double.toString(highGoalPipeline.calculatePitch(UGAngleHighGoalPipeline.Target.RED))),
        new TelemetryPacket("Goal yaw", Double.toString(highGoalPipeline.calculateYaw(UGAngleHighGoalPipeline.Target.RED))),
        new TelemetryPacket("Automatic State", automaticState.name())
        };
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
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else {
                shooterState = ShooterState.Normal;
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
                if(gameState == GameState.Intake) {
                    intake.setPower(-1);
                }
                intakeDirection = IntakeDirection.Out;
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (intakeDirection == IntakeDirection.Out) {
                if(gameState==GameState.Intake) {
                    intake.setPower(1);
                }
                intakeDirection = IntakeDirection.In;
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }
        intakePrevButtonState = intakeCurrentButtonState;
    }

    void Flick() {
        timeSinceFlicker = (System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1);
        if(timeSinceFlicker >= totalFlickTime / 2) {
            if(firstReturn) {
                flicker.setPosition(0.23);
                firstReturn = false;
            }
        }
        //For one player
        //tryFLick = gamepad1.left_trigger != 0;
        //For two players
        tryFLick = gamepad2.left_bumper || gamepad2.right_bumper;
        if (timeSinceFlicker >= totalFlickTime && tryFLick) {
            flicker.setPosition(0);
            flickerStartTime = System.nanoTime();
            firstReturn = true;
            //tryHerderDown = true;
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

        shooter.setPower(0);
        if(intakeDirection == IntakeDirection.In) {
            intake.setPower(1);
        }else {
            intake.setPower(-1);
        }
        intakeServo.setPower(1);

        tryHerderDown = true;
    }

    private void SwitchToShooting() {
        backPlate.setPosition(0);

        gameState = GameState.Shooting;
        shooter.setVelocity((shooterTargetRPM*28)/60);

        shooterStartTime = System.nanoTime();

        intakeDirection = IntakeDirection.In;
        lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        keepIntakeOn = true;
        intake.setPower(1);
        intakeStartTime = System.nanoTime();
        
        putHerdersUp();
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

    public void DriveCode() {
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        /*
        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        */
        //This is the code you would use for a non-field centric design
        if(gamepad1.left_bumper) {
            powerMultiplier = 0.25;
        } else if(gamepad1.right_bumper) {
            powerMultiplier = 0.9;
        } else {
            powerMultiplier = 0.55;
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * powerMultiplier,
                        -gamepad1.left_stick_x * powerMultiplier,
                        -gamepad1.right_stick_x * powerMultiplier
                )
        );

    }

    void DriveStateRoutine() {
        //Get the robot's current position
        Pose2d poseEstimate = drive.getPoseEstimate();
        //Update the drive
        drive.update();

        if(driveState == DriveState.DRIVER_CONTROL) {
            DriveCode();
        }else if(driveState == DriveState.NORMAL_AUTOMATIC){
            NormalAutomaticLoop(poseEstimate);
        }else if(driveState == DriveState.POWERSHOT_AUTOMATIC) {
            if(!drive.isBusy() && powerShotStep == 1) {
                powerShotStep = 2;
                drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
            } else if(!drive.isBusy() && powerShotStep ==2 ) {
                allowPowershotMachine = true;
            }
        }

        currentAState = gamepad1.a;
        currentBState = gamepad1.b;

        if(currentAState && currentAState != prevAState) {
            if(driveState == DriveState.DRIVER_CONTROL) {
                turnStep = 1;
                driveState = DriveState.NORMAL_AUTOMATIC;
                shooterState = ShooterState.Normal;
                NormalAutomaticCalculations(poseEstimate);
            } else if(driveState == DriveState.NORMAL_AUTOMATIC || driveState == DriveState.POWERSHOT_AUTOMATIC) {
                driveState = DriveState.DRIVER_CONTROL;
                allowPowershotMachine = false;
                drive.cancelFollowing();
                webcam.setPipeline(highGoalPipeline);
            }
        }

        if(currentBState && currentBState != prevBState) {
            if(driveState == DriveState.DRIVER_CONTROL) {
                driveState = DriveState.POWERSHOT_AUTOMATIC;
                shooterState = ShooterState.PowerShot;
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                webcam.setPipeline(powershotPipeline);
                turnStep = 1;
                powerShotStep = 1;
                /*
                if(currentPowerShot == Powershot.Left) currentPowerShotTargetPoint = powerShotShootingPointLeft;
                else if(currentPowerShot == Powershot.Center) currentPowerShotTargetPoint = powerShotShootingPointCenter;
                else if(currentPowerShot == Powershot.Right) currentPowerShotTargetPoint = powerShotShootingPointRight;

                Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                        .lineToLinearHeading(new Pose2d(currentPowerShotTargetPoint.x, currentPowerShotTargetPoint.y,0))
                        .build();
                */

                Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                        .lineToLinearHeading(new Pose2d(-4, -24, 0))
                        .build();
                drive.followTrajectoryAsync(trajectory);
            } else if (driveState == DriveState.POWERSHOT_AUTOMATIC || driveState == DriveState.NORMAL_AUTOMATIC) {
                driveState = DriveState.DRIVER_CONTROL;
                currentPowerShot = Powershot.Right;
                webcam.setPipeline(highGoalPipeline);
                drive.cancelFollowing();
            }
        }
        currentYState = gamepad1.y;
        if(currentYState && currentYState != prevYState) {
            if(driveState  != DriveState.ANGLEVISION) {
                drive.cancelFollowing();
                webcam.setPipeline(highGoalPipeline);
                driveState = DriveState.ANGLEVISION;
            } else if(driveState == DriveState.ANGLEVISION) {
                driveState = DriveState.DRIVER_CONTROL;
                drive.cancelFollowing();
                shouldFollow = true;
            }
        }
        prevYState = currentYState;
        if(driveState == DriveState.ANGLEVISION) {
            double yaw = highGoalPipeline.calculateYaw(UGAngleHighGoalPipeline.Target.RED);
            if(!drive.isBusy() && shouldFollow) {
                if(highGoalPipeline.isRedVisible()) {
                    if(yaw >= 0) {
                        drive.turnAsync(-Math.toRadians(yaw+leftFactor));
                    }else if(yaw <=0) {
                        drive.turnAsync(-Math.toRadians(yaw-rightFactor));
                    }
                    shouldFollow = false;
                }
            } else if(!drive.isBusy() && !shouldFollow) {
                driveState = DriveState.DRIVER_CONTROL;
            }
        }

        dpadLeft = gamepad1.dpad_left;
        dpadRight = gamepad1.dpad_right;
        dpadCurrentSate = dpadLeft || dpadRight;

        if(dpadCurrentSate && dpadCurrentSate != dpadPrevState && driveState == DriveState.POWERSHOT_AUTOMATIC && allowPowershotMachine) {
            drive.cancelFollowing();
            if(dpadLeft) {
                if(currentPowerShot == Powershot.Left) {
                    driveState = DriveState.DRIVER_CONTROL;
                    currentPowerShot = Powershot.Right;
                    SwitchToShooting();
                    shooterState = ShooterState.Normal;
                    webcam.setPipeline(highGoalPipeline);
                } else if(currentPowerShot == Powershot.Center) {
                    currentPowerShot = Powershot.Left;
                    /*
                    Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(powerShotShootingPointLeft.x, powerShotShootingPointLeft.y,0))
                            .build();
                    drive.followTrajectoryAsync(trajectory);

                     */
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.LEFT))+leftShotOffset));
                } else if(currentPowerShot == Powershot.Right) {
                    currentPowerShot = Powershot.Center;
                    /*
                    Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(powerShotShootingPointCenter.x, powerShotShootingPointCenter.y,0))
                            .build();
                    drive.followTrajectoryAsync(trajectory);

                     */
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                }
            } else if(dpadRight && false) {
                if(currentPowerShot == Powershot.Left) {
                    currentPowerShot = Powershot.Center;
                    /*
                    Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(powerShotShootingPointCenter.x, powerShotShootingPointCenter.y,0))
                            .build();
                    drive.followTrajectoryAsync(trajectory);

                     */
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                } else if(currentPowerShot == Powershot.Center) {
                    currentPowerShot = Powershot.Right;
                    /*
                    Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(powerShotShootingPointRight.x, powerShotShootingPointRight.y,0))
                            .build();
                    drive.followTrajectoryAsync(trajectory);

                     */

                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
                } else if(currentPowerShot == Powershot.Right) {
                }
            }
        }

        dpadPrevState = dpadCurrentSate;
        prevAState = currentAState;
        prevBState = currentBState;

        if(gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0 && gamepad1.x) {
            drive.setPoseEstimate(new Pose2d(62.47, 13.12, 0));
        }
    }

    private void NormalAutomaticLoop(Pose2d poseEstimate) {
        if(!drive.isBusy() && automaticState == AutomaticState.GoToLine) {
            automaticState = AutomaticState.AngleOdo;
        }else if(!drive.isBusy() && automaticState == AutomaticState.AngleOdo) {
            drive.turnAsync(Angle.normDelta(drive.headingFromPoint(currentXPos, currentYPos) - poseEstimate.getHeading()));
            automaticState = AutomaticState.AngleVision;
        } else if(!drive.isBusy() && automaticState == AutomaticState.AngleVision) {
            double yaw = highGoalPipeline.calculateYaw(UGAngleHighGoalPipeline.Target.RED);
            if(highGoalPipeline.isRedVisible()) {
                if(yaw >= 0) {
                    drive.turnAsync(-Math.toRadians(yaw+leftFactor));
                }else if(yaw <= 0) {
                    drive.turnAsync(-Math.toRadians(yaw-rightFactor));
                }
                shouldFollow = false;
            }
            automaticState = AutomaticState.Finish;
        } else if(!drive.isBusy() && automaticState == AutomaticState.Finish) {
            driveState = DriveState.DRIVER_CONTROL;
            lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
        }
    }

    private void NormalAutomaticCalculations(Pose2d poseEstimate) {
        if(poseEstimate.getY() <= goalYPos) {
            currentXPos = rightGoalXPos;
            currentYPos = rightGoalYPos;
        } else {
            currentXPos = goalXPos;
            currentYPos = goalYPos;
        }

        automaticState = AutomaticState.GoToLine;
        //This runs if we're already behing the shooing line
        //if(poseEstimate.getX() <= constraintFront && poseEstimate.getX() >= constraintBack && poseEstimate.getY() <= constraintLeft && poseEstimate.getY() >= constraintRight) {
        if(false) {
            if((poseEstimate.getHeading() <= Math.toRadians(45) || poseEstimate.getHeading() >= Math.toRadians(315)) && highGoalPipeline.isRedVisible()) {
                automaticState = AutomaticState.AngleVision;
            } else {
                automaticState = AutomaticState.AngleOdo;
            }
        } else {
            //This rusn if we're in front of the shooting line
            automaticState = AutomaticState.GoToLine;
            double yPos = poseEstimate.getY();
            if(yPos <= constraintRight) {
                yPos = constraintRight;
            }else if(yPos >= constraintLeft) {
                yPos = constraintLeft;
            }
            Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                    //.lineToLinearHeading(new Pose2d(constraintFront, yPos, drive.headingFromPoints(goalXPos, goalYPos, constraintFront, yPos)))
                    //.lineToLinearHeading(new Pose2d(-4, -24, drive.headingFromPoints(goalXPos, goalYPos, -4, -24)))
                    .lineToLinearHeading(new Pose2d(-4, -24, Math.toRadians(351)))
                    .build();
            drive.followTrajectoryAsync(trajectory);
        }
    }

    private void InitializeHardwaremap() {
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

        //For the ring herder
        herderServoLeft = hardwareMap.servo.get("herder_left");
        herderServoRight = hardwareMap.servo.get("herder_right");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights2 = hardwareMap.get(RevBlinkinLedDriver.class, "lights2");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all the lights and servos to the correct positions to get ready for the beginning of the game
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        //Open Wobble Claw
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);

        backPlate.setPosition(1);
        flicker.setPosition(0.23);

        wobbleLifter.setPosition(wobbleLifterUpPos);
        ringKnocker.setPosition(0);

        herderServoLeft.setPosition(0);
        herderServoRight.setPosition(1);
    }

    private enum GameState {Shooting, Intake}

    private enum IntakeDirection {In, Out}

    private enum ShooterState {Normal, PowerShot}

    private enum ClawState {Open, Closed}

    private enum DriveState {DRIVER_CONTROL, NORMAL_AUTOMATIC, POWERSHOT_AUTOMATIC, ANGLEVISION}

    private enum Powershot {Left, Center, Right}

    private enum WobbleState {Initial, DownOpen, VerticalClosed, DropWobble, RetractedClosed}

    private enum HerderState { Down, Up}

    private enum AutomaticState {GoToLine, AngleOdo, AngleVision, Finish}

}
