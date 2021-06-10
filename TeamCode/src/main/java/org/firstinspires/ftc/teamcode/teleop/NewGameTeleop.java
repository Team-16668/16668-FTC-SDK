package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.roadrunner.util.RoadrunnerPoint;
import org.firstinspires.ftc.teamcode.roadrunner.util.TelemetryPacket;
import org.firstinspires.ftc.teamcode.vision.CustomPowershotPipelineRed;
import org.firstinspires.ftc.teamcode.vision.UGAngleHighGoalPipeline;
import org.firstinspires.ftc.teamcode.parts.WobbleTurret;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name= "Game Teleop")
public class NewGameTeleop extends LinearOpMode {
    //Initial Variable initialization
    //Hardware devices
    RevBlinkinLedDriver lights, lights2;
    DcMotor intake;
    DcMotorEx shooter;
    Servo backPlate, flicker, wobbleLifter, ringKnocker, herderServoLeft, herderServoRight;
    CRServo intakeServo;
    private VoltageSensor batteryVoltageSensor;

    WobbleTurret turret;

    boolean allowPowershotMachine = false;
    int powerShotStep = 1;

    //All the State Machines
    GameState gameState = GameState.Intake;
    DriveState driveState = DriveState.DRIVER_CONTROL;
    Powershot currentPowerShot = Powershot.Right;
    IntakeDirection intakeDirection = IntakeDirection.In;
    ShooterState shooterState = ShooterState.Normal;
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
    public static double powerShotTargetRPM = 3210;
    double shooterStartTime,
            shooterTargetRPM = normalTargetRPM;
    InterpLUT shooterLut;

    //Logic for Power Shots
    boolean powerShotCurrentButton = false;
    boolean powerShotPrevButton = false;

    //Logic for Wobble Claw and Arm
    boolean prevWobbleDpad = false;
    boolean currWobbleDpad = false;

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

        //Set the Localizer/Positioning System
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                190, 0, 10, 13 /* * 12 / batteryVoltageSensor.getVoltage() */
        ));

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        //drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));
        drive.setPoseEstimate(PoseStorage.currentPose);

        turret = new WobbleTurret(hardwareMap, WobbleTurret.TurretState.STOWED, 0.5);
        turret.init();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Put the wobble arm down by default
        intake.setPower(1);
        intakeServo.setPower(1);
        shooterStartTime = System.nanoTime();
        flickerStartTime = System.nanoTime();



        while(opModeIsActive()) {

            drive.drawBoundingBox(constraintLeft, constraintRight, constraintBack, constraintFront);

            //Do all functions to run the motors at the right speeds
            DriveStateRoutine();

            //Claw and Wobble Arm Code
            TurretSubroutine();

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

    public void TurretSubroutine() {
        turret.update();

        currWobbleDpad = gamepad2.dpad_left;
        if(currWobbleDpad && currWobbleDpad != prevWobbleDpad) {
            turret.advanceStateMachineTeleOp();
        }
        prevWobbleDpad = currWobbleDpad;

        if(gamepad2.dpad_right) {
            turret.setState(WobbleTurret.TurretState.STOWED);
        }

        if(gamepad2.dpad_down) {
            turret.wobbleLinearMotor.setPower(0);
            turret.wobbleRotaryMotor.setPower(0);
            turret.clawLift.setPower(0);
        }

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
        new TelemetryPacket("Automatic State", automaticState.name()),
        new TelemetryPacket("Wobble Goal State", turret.turretState.name()),
        new TelemetryPacket("Rotary Power", Double.toString(turret.wobbleRotaryMotor.getPower())),
        new TelemetryPacket("Linear Power", Double.toString(turret.wobbleLinearMotor.getPower())),
        new TelemetryPacket("Lift Power", Double.toString(turret.clawLift.getPower()))
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
                if(gameState== GameState.Intake) {
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
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.LEFT))+leftShotOffset));
                } else if(currentPowerShot == Powershot.Right) {
                    currentPowerShot = Powershot.Center;
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                }
            } else if(dpadRight && false) {
                if(currentPowerShot == Powershot.Left) {
                    currentPowerShot = Powershot.Center;
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                } else if(currentPowerShot == Powershot.Center) {
                    currentPowerShot = Powershot.Right;

                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
                } else if(currentPowerShot == Powershot.Right) {
                    drive.turnAsync(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
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
            if(!(poseEstimate.getHeading() < 15) || !(poseEstimate.getHeading() > Math.toRadians(345))) {
                drive.turnAsync(Angle.normDelta(drive.headingFromPoint(currentXPos, currentYPos) - poseEstimate.getHeading()));
            }
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
                    .lineToLinearHeading(new Pose2d(-4, -24, Math.toRadians(350)))
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

        //For the Flicker and Backplate
        backPlate = hardwareMap.servo.get("backplate");
        flicker = hardwareMap.servo.get("flicker");

        //For the ring herder
        herderServoLeft = hardwareMap.servo.get("herder_left");
        herderServoRight = hardwareMap.servo.get("herder_right");

        //For the wobble lifter
        wobbleLifter = hardwareMap.servo.get("wobble_lifter");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights2 = hardwareMap.get(RevBlinkinLedDriver.class, "lights2");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Set all the lights and servos to the correct positions to get ready for the beginning of the game
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

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

    private enum DriveState {DRIVER_CONTROL, NORMAL_AUTOMATIC, POWERSHOT_AUTOMATIC, ANGLEVISION}

    private enum Powershot {Left, Center, Right}

    private enum HerderState { Down, Up}

    private enum AutomaticState {GoToLine, AngleOdo, AngleVision, Finish}

}
