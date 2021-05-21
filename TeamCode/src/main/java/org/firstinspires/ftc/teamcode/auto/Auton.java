package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tools.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.CustomPowershotPipelineRed;
import org.firstinspires.ftc.teamcode.parts.WobbleTurret;
import org.firstinspires.ftc.teamcode.parts.WobbleTurret.TurretState;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import static org.firstinspires.ftc.teamcode.parts.WobbleTurret.TurretState.*;

/*
 * This is an example ofwqa a more complex path to really test the tuning.
 */
@Autonomous(name = "Auton", group = "drive")
public class Auton extends LinearOpMode {

    //Ultimate Goal Specific Hardware
    DcMotor intake;
    DcMotorEx shooter;
    Servo backPlate, flicker, ringKnocker, herderServoLeft, herderServoRight;
    CRServo intakeServo;
    RevBlinkinLedDriver lights, lights2;

    WobbleTurret turret;

    SampleMecanumDrive drive;

    ElapsedTime timer;

    OpenCvWebcam topcam;
    OpenCvWebcam bottomCam;
    OpenCvSwitchableWebcam switchableWebcam;
    SkystoneDeterminationPipeline ringPipeline;
    CustomPowershotPipelineRed powershotPipeline;

    double flickerStartTime;

    double wobbleLifterUpPos = 0.649;

    public static double leftShotOffset  = -2;
    public static double centerShotOffset = -3;
    public static double rightShotOffset = -4;


    @Override
    public void runOpMode() throws InterruptedException {
            setTelemetryData("Status", "Initializing");

            drive = new SampleMecanumDrive(hardwareMap);

            drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));

            timer = new ElapsedTime();
            //Wonderful timer commands that are probably a lot better than what I have now.
            /*
            timer.reset();
            timer.seconds();
             */

            turret = new WobbleTurret(hardwareMap, STOWED);
            turret.init();

            defineHardware();
            initialize();

            //Zero Ring Trajectories
            setTelemetryData("Status", "Building zero rings trajectories");
            Trajectory startToPowershotPos = drive.trajectoryBuilder(new Pose2d(-62, -26, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-4, -24,0))
                    .build();

            //Trajectory powerShotToWobbleZeroRing = drive.trajectoryBuilder(powershotPosToRightPowershot.end())
            Trajectory powerShotToWobbleZeroRing = drive.trajectoryBuilder(startToPowershotPos.end())
                    .lineToLinearHeading(new Pose2d(-6, -61, Math.toRadians(335)))
                    .build();

            Trajectory wobbleToWobble2ZeroRing = drive.trajectoryBuilder(new Pose2d(powerShotToWobbleZeroRing.end().getX(), -57, powerShotToWobbleZeroRing.end().getHeading()))
                    .splineToSplineHeading(new Pose2d(-10, -55, 0), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-31, -55, 0),  Math.toRadians(180),  new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory dropWobble2ZeroRing = drive.trajectoryBuilder(wobbleToWobble2ZeroRing.end())
                    .lineToLinearHeading(new Pose2d(-9, -59, Math.toRadians(315)))
                    .build();

            Trajectory pickUpRingsZeroRings = drive.trajectoryBuilder(dropWobble2ZeroRing.end())
                    .splineToConstantHeading(new Vector2d(-14, -50), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        lowerBackplate();
                        runIntakeForward();
                    })
                    .splineToSplineHeading(new Pose2d(0, -10, Math.toRadians(0)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(50, -12, 0), 0)
                    .addDisplacementMarker(() -> {setShooterRPM(3650);})
                    .splineToSplineHeading(new Pose2d(-4, -24, Math.toRadians(345)), 0)
                    .addDisplacementMarker(() -> {
                        liftBackplate(); stopIntake();
                    })
                    .build();

            Trajectory park = drive.trajectoryBuilder(pickUpRingsZeroRings.end())
                    .lineToLinearHeading(new Pose2d(9, -24, 0))
                    .build();

            //1 Ring Trajectories
            setTelemetryData("Status", "Building one ring trajectories");

            Trajectory shootInitialOneRing = drive.trajectoryBuilder(new Pose2d(-62, -26, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-36, -38, Math.toRadians(358)))
                    .build();

            Trajectory pickUpOneRing = drive.trajectoryBuilder(shootInitialOneRing.end())
                    .addDisplacementMarker(() -> {
                        runIntakeForward();
                        setShooterRPM(3200);
                        lowerBackplate();
                    })
                    .splineToSplineHeading(new Pose2d(12, -36, 0), 0,  new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToLinearHeading(new Pose2d(-4, -24, 0), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        liftBackplate();
                        stopIntake();
                    })
                    .build();

            Trajectory wobbleOneRing = drive.trajectoryBuilder(pickUpOneRing.end())
                    .addDisplacementMarker(() -> {
                        lowerBackplate();
                        runIntakeForward();
                    })
                    .lineToLinearHeading(new Pose2d(21, -50, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {stopIntake();})
                    .build();

            Trajectory goToWobble2OneRing = drive.trajectoryBuilder(wobbleOneRing.end())
                    .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-10, -55, 0), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-31, -55, 0),  Math.toRadians(180),  new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();


            Trajectory dropWobble2OneRing = drive.trajectoryBuilder(goToWobble2OneRing.end())
                    .addDisplacementMarker(() -> {
                        lowerBackplate();
                        setShooterRPM(0);
                    })
                    .lineToLinearHeading(new Pose2d(17, -50, 0))
                    .build();

            Trajectory pickUpRingsOneRing = drive.trajectoryBuilder(dropWobble2OneRing.end())
                    .splineToConstantHeading(new Vector2d(-14, -50), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        lowerBackplate();
                        runIntakeForward();
                    })
                    .splineToSplineHeading(new Pose2d(0, -12, Math.toRadians(0)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(50, -12, 0), 0)
                    .addDisplacementMarker(() -> {setShooterRPM(3650);})
                    .splineToSplineHeading(new Pose2d(-4, -24, Math.toRadians(345)), 0)
                    .addDisplacementMarker(() -> {
                        liftBackplate(); stopIntake();
                    })
                    .build();

            Trajectory parkOneRing = drive.trajectoryBuilder(pickUpRingsOneRing.end())
                    .lineToLinearHeading(new Pose2d(12, -24, 0))
                    .build();

            //4 Ring Trajectories
            setTelemetryData("Status", "Building four ring trajectories");

            Trajectory shootInitialFourRing = drive.trajectoryBuilder(new Pose2d(-62, -26, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                        setShooterRPM(3675);
                    })
                    .lineToLinearHeading(new Pose2d(-40, -37, Math.toRadians(358)))
                    .build();

            Trajectory pickUpOneRingFourRings = drive.trajectoryBuilder(shootInitialFourRing.end())
                    .addDisplacementMarker(() -> {
                        lowerBackplate();
                        runIntakeForward();
                        setShooterRPM(3600);
                    })
                    .lineToLinearHeading(new Pose2d(-30, -37, 0),  new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory pickUpNextRingsFourRings = drive.trajectoryBuilder(pickUpOneRingFourRings.end())
                    .addDisplacementMarker(() -> {lowerBackplate(); runIntakeForward(); setShooterRPM(3200);})
                    .splineToSplineHeading(new Pose2d(-4, -37, 0), 0, new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(new Vector2d(-4, -24), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        liftUpRingKnocker();
                    })
                    .build();

            Trajectory deliverWobbleFourRing = drive.trajectoryBuilder(pickUpNextRingsFourRings.end())
                    .lineToLinearHeading(new Pose2d(42, -60, Math.toRadians(345)), new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(62, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> {lowerBackplate();})
                    .build();

            Trajectory wobbleToWobble2FourRing = drive.trajectoryBuilder(new Pose2d(deliverWobbleFourRing.end().getX()-10, deliverWobbleFourRing.end().getY(), deliverWobbleFourRing.end().getHeading()))
                    .splineToSplineHeading(new Pose2d(-10, -56, 0), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-31, -56, 0),  Math.toRadians(180),  new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            Trajectory deliverWobble2FourRing = drive.trajectoryBuilder(wobbleToWobble2FourRing.end())
                .splineToConstantHeading(new Vector2d(0, -58), 0)
                .splineToSplineHeading(new Pose2d(40, -58, Math.toRadians(315)), Math.toRadians(315))
                .build();

            Trajectory park4Ring = drive.trajectoryBuilder(deliverWobble2FourRing.end())
                .lineToLinearHeading(new Pose2d(6, -24, 0))
                .build();

            //Webcam initialization
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

            topcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "goal_webcam"), viewportContainerIds[0]);
            bottomCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), viewportContainerIds[1]);
            ringPipeline = new SkystoneDeterminationPipeline();
            powershotPipeline = new CustomPowershotPipelineRed(60, 0, 0);

            bottomCam.setPipeline(ringPipeline);
            topcam.setPipeline(powershotPipeline);

            bottomCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    bottomCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
            topcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    topcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });

            telemetry.addData("Status", "Waiting for start");
            telemetry.update();

            while(!opModeIsActive() && !isStopRequested()) {
                if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }else if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }else if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }

                sleep(50);
            }

            waitForStart();

            if (isStopRequested()) return;

            ringKnocker.setPosition(1);
            sleep(500);

            telemetry.addData("Analysis", ringPipeline.getAnalysis());
            telemetry.addData("Position", ringPipeline.position);
            telemetry.update();

            if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                //Zero Rings
                bottomCam.stopStreaming();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                //One Ring
                bottomCam.stopStreaming();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
                //Four Rings
                bottomCam.stopStreaming();
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                setShooterRPM(3190);
                liftUpRingKnocker();
                drive.followTrajectory(startToPowershotPos);
                sleep(0);
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
                Flick();
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                Flick();
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.LEFT))+leftShotOffset));
                Flick();
                setShooterRPM(0);
                topcam.stopStreaming();
                lowerBackplate();
                CustomFollowDrop(powerShotToWobbleZeroRing);
                DropWobble();
                followAsyncArm(wobbleToWobble2ZeroRing, BACK_OPEN, 0.5);
                turret.claw.setPosition(turret.clawClosed);
                sleep(500);
                CustomFollowDrop(dropWobble2ZeroRing);
                turret.claw.setPosition(turret.clawOpen);
                sleep(500);

                RetractArm(pickUpRingsZeroRings);
                Flick();
                Flick();
                Flick();
                Flick();
                drive.followTrajectory(park);
            } else if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                setShooterRPM(3675);
                liftUpRingKnocker();
                intakeServo.setPower(1);
                drive.followTrajectory(shootInitialOneRing);
                intakeServo.setPower(0);
                sleep(500);
                Flick();
                drive.followTrajectory(pickUpOneRing);
                sleep(0);
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
                Flick();
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                Flick();
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.LEFT))+leftShotOffset));
                Flick();
                setShooterRPM(0);
                topcam.stopStreaming();
                CustomFollowDrop(wobbleOneRing);
                DropWobble();
                followAsyncArm(goToWobble2OneRing, BACK_OPEN, 0.5);
                turret.claw.setPosition(turret.clawClosed);
                sleep(500);
                CustomFollowDrop(dropWobble2OneRing);
                turret.claw.setPosition(turret.clawOpen);
                sleep(500);
                RetractArm(parkOneRing);
            }else if(ringPipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
                drive.followTrajectory(shootInitialFourRing);
                Flick();
                Flick();
                Flick();
                Flick();
                drive.followTrajectory(pickUpOneRingFourRings);
                sleep(750);
                liftBackplate();
                stopIntake();
                sleep(500);
                Flick();
                Flick();
                setShooterRPM(3200);
                drive.followTrajectory(pickUpNextRingsFourRings);
                sleep(1000);
                liftBackplate();
                stopIntake();
                sleep(500);
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT))+rightShotOffset));
                Flick();
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER))+centerShotOffset));
                Flick();
                drive.turn(Math.toRadians((-powershotPipeline.calculateYaw(CustomPowershotPipelineRed.Target.LEFT))+leftShotOffset));
                Flick();
                CustomFollowDrop(deliverWobbleFourRing);
                DropWobble();
                setShooterRPM(0);
                followAsyncArm(wobbleToWobble2FourRing, BACK_OPEN, 0.5);
                turret.claw.setPosition(turret.clawClosed);
                sleep(500);
                CustomFollowDrop(deliverWobble2FourRing);
                turret.claw.setPosition(turret.clawOpen);
                sleep(500);

                RetractArm(park4Ring);
            }

            PoseStorage.currentPose = drive.getPoseEstimate();

            timer.reset();

            while(opModeIsActive() && timer.seconds() < 2) {
                turret.linearInCheck();
                turret.rotaryForwardCheck();
                turret.liftUpCheck();
                sleep(10);
            }
    }

    private void defineHardware() {
        //Ultimate Goal Specific Hardware
        //Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        //Intake
        intake = hardwareMap.dcMotor.get("intake");

        ringKnocker = hardwareMap.servo.get("ring_knocker");

        //For the Flicker and Backplate
        backPlate = hardwareMap.servo.get("backplate");
        flicker = hardwareMap.servo.get("flicker");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        //For the ring herder
        herderServoLeft = hardwareMap.servo.get("herder_left");
        herderServoRight = hardwareMap.servo.get("herder_right");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights2 = hardwareMap.get(RevBlinkinLedDriver.class, "lights2");
    }

    public void initialize() {
        //Set motor directions
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        //Set servo states
        backPlate.setPosition(0);
        flicker.setPosition(0.23);
        herderServoLeft.setPosition(0);
        herderServoRight.setPosition(1);

        ringKnocker.setPosition(0);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                190, 0, 10, 13 /* * 12 / batteryVoltageSensor.getVoltage() */
        ));
    }

    public void followAsyncArm(Trajectory traj, TurretState state, double delay) {
        drive.followTrajectoryAsync(traj);

        timer.reset();

        while(opModeIsActive() && (drive.isBusy() || ((turret.busy || timer.seconds()<=delay)))) {
            if(timer.seconds()>delay) {
                turret.setState(state);
            }

            turret.update();

            drive.update();
            telemetry.addData("time", turret.timer.seconds());
            telemetry.update();
        }
    }

    public void CustomFollowDrop(Trajectory traj) {
        drive.followTrajectoryAsync(traj);

        turret.tryLinearForward();
        turret.tryRotaryForward();

        while(opModeIsActive() && drive.isBusy()) {
            drive.update();

            turret.linearOutCheck();
            turret.rotaryForwardCheck();
        }
    }

    public void RetractArm(Trajectory traj) {
        drive.followTrajectoryAsync(traj);

        turret.tryLinearBackward();
        turret.tryRotaryForward();
        turret.tryLiftUp();

        while(opModeIsActive() && drive.isBusy()) {
            drive.update();

            turret.linearInCheck();
            turret.rotaryForwardCheck();
            turret.liftUpCheck();
        }
    }

    public void DropWobble() {
        timer.reset();
        turret.tryLiftDown();
        while(timer.seconds() < 1) {
            turret.liftDownCheck();
        }
        turret.clawLift.setPower(0);
        turret.claw.setPosition(turret.clawOpen);
        sleep(500);
    }


    void Flick() {
        flickerStartTime = System.nanoTime();
        flicker.setPosition(0);
        while((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= 0.25 && opModeIsActive() && !isStopRequested()) {
            sleep(10);
        }
        flicker.setPosition(0.23);
        while((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= 0.5 && opModeIsActive() && !isStopRequested()) {
            sleep(10);
        }
    }

    public void setShooterRPM(double RPM) {
        shooter.setVelocity((RPM/60)*28);
    }

    public void runIntakeForward() {
        intake.setPower(1);
        intakeServo.setPower(1);
    }

    public void runIntakeBackward() {
        intake.setPower(-1);
        intakeServo.setPower(-1);
    }

    public void stopIntake() {
        intake.setPower(0);
        intakeServo.setPower(0);
    }

    public void liftBackplate() {
        backPlate.setPosition(0);
    }

    public void lowerBackplate() {
        backPlate.setPosition(1);
    }

    public void putDownRingKnocker() {
        ringKnocker.setPosition(1);
    }

    public void liftUpRingKnocker() {
        ringKnocker.setPosition(0);
    }

    public void setLightPattern(int lightSet, RevBlinkinLedDriver.BlinkinPattern pattern) {
        if(lightSet == 1) {
            lights.setPattern(pattern);
        } else if (lightSet == 2) {
            lights2.setPattern(pattern);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(65,115);

        static final int REGION_WIDTH = 45;
        static final int REGION_HEIGHT = 55;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition position = OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    private void setTelemetryData(String heading, String data) {
        telemetry.addData(heading, data);
        telemetry.update();
    }
}