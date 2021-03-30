package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Roadrunner.util.RoadrunnerPoint;

import java.util.concurrent.TimeUnit;

@TeleOp(name= "Roadrunner TeleOp")
public class RoadrunnerTeleOp extends LinearOpMode {
    //Initial Variable initialization
    //Hardware devices
    RevBlinkinLedDriver lights, lights2;
    DcMotor intake, wobbleArm;
    DcMotorEx shooter;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, wobbleLifter, ringKnocker;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;
    private VoltageSensor batteryVoltageSensor;

    //All the State Machines
    GameState gameState = GameState.Intake;
    DriveState driveState = DriveState.DRIVER_CONTROL;
    Powershot currentPowerShot = Powershot.Left;
    IntakeDirection intakeDirection = IntakeDirection.In;
    ShooterState shooterState = ShooterState.Normal;
    ClawState clawState = ClawState.Closed;
    WobbleState wobbleState = WobbleState.Initial;



    //Global Game State Variable
    FtcDashboard dashboard = FtcDashboard.getInstance();
    double goalXPos = 72;
    double goalYPos = -39;
    double rightGoalXPos = 72;
    double rightGoalYPos = -37;
    double currentXPos = goalXPos;
    double currentYPos = goalYPos;
    double shootingLineX = -10;
    boolean currentBState = false;
    boolean prevBState = false;
    boolean dpadLeft;
    boolean dpadRight;
    boolean dpadCurrentSate;
    boolean dpadPrevState;
    RoadrunnerPoint powerShotShootingPoint = new RoadrunnerPoint(-10, -10);
    RoadrunnerPoint powerShotTargetPointLeft = new RoadrunnerPoint(72, -10);
    RoadrunnerPoint powerShotTargetPointCenter = new RoadrunnerPoint(72, -20);
    RoadrunnerPoint powerShotTargetPointRight = new RoadrunnerPoint(72, -30);
    RoadrunnerPoint currentPowerShotTargetPoint = new RoadrunnerPoint(powerShotTargetPointLeft.x, powerShotTargetPointLeft.y);

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

    //Logic for Shooter
    double shooterStartTime,
            normalTargetRPM = 4400,
            powerShotTargetRPM = 3800,
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

    int turnStep = 1;

    SampleMecanumDriveCancelable drive;

    public void runOpMode() {

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
        shooterLut.add(200,4350);
        shooterLut.createLUT();

        //Set the Localizer/Positioning System
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                50, 0, 10, 15 * 12 / batteryVoltageSensor.getVoltage()
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
                //shooter.setVelocity((shooterTargetRPM*28)/60);
                Shooting();

            } else {
                Intake();
            }

            Flick();

            NormalToPowerShot();

            if(keepIntakeOn) {
                IntakeTimer();
            }

            Telemetry();
        }

    }

    private void Shooting() {
        if(shooterState == ShooterState.Normal) {
            Pose2d myPose = drive.getPoseEstimate();
            shooterTargetRPM = drive.distanceFromPoint(myPose.getX(), myPose.getY(), goalXPos, goalYPos);
            double shooterVelocity = shooterLut.get(drive.distanceFromPoint(myPose.getX(), myPose.getY(), goalXPos, goalYPos));
            //double shooterVelocity = shooterLut.get(4);
            shooter.setVelocity((shooterVelocity*28)/60);
        } else if(shooterState == ShooterState.PowerShot) {
            shooterTargetRPM = powerShotTargetRPM;
            shooter.setVelocity((powerShotTargetRPM*28)/60);
        }
    }

    void Telemetry() {
        Pose2d myPose = drive.getPoseEstimate();

        telemetry.addData("velocity", (shooter.getVelocity()/28)*60);
        telemetry.addData("target velocity", shooterTargetRPM);
        telemetry.addData("shooter mode", shooterState);
        telemetry.addData("Drive mode", driveState);
        telemetry.addData("Powershot", currentPowerShot);
        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", myPose.getHeading());
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
            if(!drive.isBusy() && turnStep == 1) {
                drive.turnAsync(Angle.normDelta(drive.headingFromPoint(currentXPos, currentYPos) - poseEstimate.getHeading()));
                turnStep = 2;
            } else if(!drive.isBusy() && turnStep == 2) {
                turnStep = 1;
                driveState = DriveState.DRIVER_CONTROL;
            }
        }else if(driveState == DriveState.POWERSHOT_AUTOMATIC) {
            if(!drive.isBusy() && turnStep == 1) {
                turnStep = 2;
                drive.turnAsync(Angle.normDelta(drive.headingFromPoint(currentPowerShotTargetPoint.x, currentPowerShotTargetPoint.y)
                        - poseEstimate.getHeading()));
            }else if(!drive.isBusy() && turnStep == 2) {
                //turnStep = 1;
            }
        }

        currentAState = gamepad1.a;
        currentBState = gamepad1.b;

        if(currentAState && currentAState != prevAState) {
            if(driveState == DriveState.DRIVER_CONTROL) {
                turnStep = 1;
                driveState = DriveState.NORMAL_AUTOMATIC;

                if(poseEstimate.getY() <= goalYPos) {
                    currentXPos = rightGoalXPos;
                    currentYPos = rightGoalYPos;
                } else {
                    currentXPos = goalXPos;
                    currentYPos = goalYPos;
                }

                if(poseEstimate.getX() <= shootingLineX) {
                    drive.turnAsync(Angle.normDelta(drive.headingFromPoint(currentXPos, currentYPos) - poseEstimate.getHeading()));
                } else {
                    Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(shootingLineX, poseEstimate.getY(), drive.headingFromPoints(currentXPos, currentYPos, shootingLineX, poseEstimate.getY())))
                            .build();
                    drive.followTrajectoryAsync(trajectory);
                }
            } else if(driveState == DriveState.NORMAL_AUTOMATIC || driveState == DriveState.POWERSHOT_AUTOMATIC) {
                driveState = DriveState.DRIVER_CONTROL;
                drive.cancelFollowing();
            }
        }

        if(currentBState && currentBState != prevBState) {
            if(driveState == DriveState.DRIVER_CONTROL) {
                driveState = DriveState.POWERSHOT_AUTOMATIC;
                turnStep = 1;

                if(currentPowerShot == Powershot.Left) currentPowerShotTargetPoint = powerShotTargetPointLeft;
                else if(currentPowerShot == Powershot.Center) currentPowerShotTargetPoint = powerShotTargetPointCenter;
                else if(currentPowerShot == Powershot.Right) currentPowerShotTargetPoint = powerShotTargetPointRight;

                Trajectory trajectory = drive.trajectoryBuilder(poseEstimate)
                        .lineToLinearHeading(new Pose2d(powerShotShootingPoint.x, powerShotShootingPoint.y,
                                drive.headingFromPoints(currentPowerShotTargetPoint.x, currentPowerShotTargetPoint.y,
                                        powerShotShootingPoint.x, powerShotShootingPoint.y)))
                        .build();

                drive.followTrajectoryAsync(trajectory);
            } else if (driveState == DriveState.POWERSHOT_AUTOMATIC || driveState == DriveState.NORMAL_AUTOMATIC) {
                driveState = DriveState.DRIVER_CONTROL;
                drive.cancelFollowing();
            }
        }

        dpadLeft = gamepad1.dpad_left;
        dpadRight = gamepad1.dpad_right;
        dpadCurrentSate = dpadLeft || dpadRight;

        if(dpadCurrentSate && dpadCurrentSate != dpadPrevState) {
            drive.cancelFollowing();
            if(dpadLeft) {
                if(currentPowerShot == Powershot.Left) {
                    //Do nothing
                } else if(currentPowerShot == Powershot.Center) {
                    currentPowerShot = Powershot.Left;
                    drive.turnAsync(Angle.normDelta(drive.headingFromPoint(powerShotTargetPointLeft.x, powerShotTargetPointLeft.y)
                            - poseEstimate.getHeading()));
                } else if(currentPowerShot == Powershot.Right) {
                    currentPowerShot = Powershot.Center;
                    drive.turnAsync(Angle.normDelta(drive.headingFromPoint(powerShotTargetPointCenter.x, powerShotTargetPointCenter.y)
                            - poseEstimate.getHeading()));
                }
            } else if(dpadRight) {
                if(currentPowerShot == Powershot.Left) {
                    currentPowerShot = Powershot.Center;
                    drive.turnAsync(Angle.normDelta(drive.headingFromPoint(powerShotTargetPointCenter.x, powerShotTargetPointCenter.y)
                            - poseEstimate.getHeading()));
                } else if(currentPowerShot == Powershot.Center) {
                    currentPowerShot = Powershot.Right;
                    drive.turnAsync(Angle.normDelta(drive.headingFromPoint(powerShotTargetPointRight.x, powerShotTargetPointRight.y)
                            - poseEstimate.getHeading()));
                } else if(currentPowerShot == Powershot.Right) {
                    //Do nothing
                }
            }
        }

        dpadPrevState = dpadCurrentSate;
        prevAState = currentAState;
        prevBState = currentBState;

        if(gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0 && gamepad1.x) {
            drive.setPoseEstimate(new Pose2d(60.5, 14.5, 0));
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
        flicker.setPosition(1);

        wobbleLifter.setPosition(0.66);
        ringKnocker.setPosition(0);
    }

    private enum GameState {Shooting, Intake}

    private enum IntakeDirection {In, Out}

    private enum ShooterState {Normal, PowerShot}

    private enum ClawState {Open, Closed}

    private enum DriveState {DRIVER_CONTROL, NORMAL_AUTOMATIC, POWERSHOT_AUTOMATIC}

    private enum Powershot {Left, Center, Right}

    private enum WobbleState {Initial, DownOpen, VerticalClosed, DropWobble, RetractedClosed}

}
