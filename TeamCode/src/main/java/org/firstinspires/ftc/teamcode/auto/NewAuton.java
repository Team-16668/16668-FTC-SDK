package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RandomTools.Vision.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.Roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class NewAuton extends LinearOpMode {

    //Ultimate Goal Specific Hardware
    DcMotor intake, wobbleArm;
    DcMotorEx shooter;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, ringKnocker, wobbleLifter;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;
    RevBlinkinLedDriver lights, lights2;

    SampleMecanumDrive drive;

    ElapsedTime timer;

    OpenCvWebcam webcam;
    SkystoneDeterminationPipeline pipeline;

    double flickerStartTime;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));

        timer = new ElapsedTime();
        //Wonderful timer commands that are probably a lot better than what I have now.
        timer.reset();
        timer.seconds();

        defineHardware();
        initialize();
        //Zero ring trajectories
        Trajectory startToPowershotPos = drive.trajectoryBuilder(new Pose2d(-62, -26, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-10, -11, /*drive.headingFromPoints(72, -10,-10, -10)*/0))
                .build();
        telemetry.addData("Status", "1");
        telemetry.update();
        Trajectory powershotPosToCenterPowershot = drive.trajectoryBuilder(startToPowershotPos.end())
                .lineToLinearHeading(new Pose2d(-10, -19, /*drive.headingFromPoints(72, -10,-10, -10)*/0))
                .build();

        Trajectory powershotPosToRightPowershot = drive.trajectoryBuilder(powershotPosToCenterPowershot.end())
                .lineToLinearHeading(new Pose2d(-10, -27, /*drive.headingFromPoints(72, -10,-10, -10)*/0))
                .build();

        Trajectory powerShotToWobble = drive.trajectoryBuilder(powershotPosToRightPowershot.end())
                .lineToLinearHeading(new Pose2d(15, -53, Math.toRadians(90)))
                .build();

        Trajectory wobbleToWobble2 = drive.trajectoryBuilder(powerShotToWobble.end())
                .splineToLinearHeading(new Pose2d(15, -45, Math.toRadians(90)), 0)
                .addDisplacementMarker(() -> { putWobbleLifterDown(); })
                .splineToSplineHeading(new Pose2d(-22, -46, Math.toRadians(270)), 0)
                .splineToSplineHeading(new Pose2d(-40, -46, Math.toRadians(270)),  0)
                .build();

        Trajectory dropWobble2 = drive.trajectoryBuilder(wobbleToWobble2.end())
                .lineToLinearHeading(new Pose2d(7, -45, 0))
                .build();

        Trajectory getAwayFromWobble2 = drive.trajectoryBuilder(dropWobble2.end())
                .splineToConstantHeading(new Vector2d(3, -30), 0)
                .addDisplacementMarker(() -> {
                    putWobbleLifterUp();
                    runIntakeForward();
                    setShooterRPM(4400);
                    grabWobble();})
                .splineToSplineHeading(new Pose2d(56, -39, Math.toRadians(90)), 0)
                .splineToConstantHeading(new Vector2d(56, 0), 0)
                .splineToSplineHeading(new Pose2d(-4, -36, Math.toRadians(355)), 0)
                .addDisplacementMarker(() -> { liftBackplate(); stopIntake();})
                .build();

        Trajectory park = drive.trajectoryBuilder(getAwayFromWobble2.end())
                .lineToConstantHeading(new Vector2d(0, -36))
                .build();

        telemetry.addData("Status", "2");
        telemetry.update();

        //4 Ring Trajectories
        Trajectory startToShootingPos = drive.trajectoryBuilder(new Pose2d(-62, -26, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-37, -35.5), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(startToShootingPos.end())
                .lineToLinearHeading(new Pose2d(45, -40, Math.toRadians(135)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-35, -42, Math.toRadians(270)), 0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(57, -45, Math.toRadians(0)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(8, -38, Math.toRadians(0)), 0)
                .build();
        telemetry.addData("Status", "3");
        telemetry.update();


        //Webcam initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        while(!opModeIsActive() && !isStopRequested()) {
            if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        ringKnocker.setPosition(1);
        sleep(500);

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
            //Zero Rings
            webcam.stopStreaming();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
            //One Ring
            webcam.stopStreaming();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            //Four Rings
            webcam.stopStreaming();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            lights2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
            setShooterRPM(3650);
            liftUpRingKnocker();
            drive.followTrajectory(startToPowershotPos);
            Flick();
            drive.followTrajectory(powershotPosToCenterPowershot);
            Flick();
            drive.followTrajectory(powershotPosToRightPowershot);
            Flick();
            setShooterRPM(0);
            lowerBackplate();
            followAsyncArm(powerShotToWobble, -0.45);
            releaseWobble();
            sleep(500);
            followAsyncArm(wobbleToWobble2, 0.55);
            liftWobble();
            sleep(500);
            drive.followTrajectory(dropWobble2);
            putWobbleLifterDown();
            sleep(500);
            drive.followTrajectory(getAwayFromWobble2);
            sleep(500);
            Flick();
            Flick();
            Flick();
            lowerBackplate();
            drive.followTrajectory(park);
        } else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {

        } else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            drive.followTrajectory(startToShootingPos);
            Thread.sleep(2000);
            drive.followTrajectory(traj2);
            Thread.sleep(2000);
            drive.followTrajectory(traj3);
            Thread.sleep(2000);
            drive.followTrajectory(traj4);
            Thread.sleep(2000);
            drive.followTrajectory(traj5);
            Thread.sleep(2000);
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void defineHardware() {
        //Ultimate Goal Specific Hardware
        //Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        //Intake
        intake = hardwareMap.dcMotor.get("intake");

        ringKnocker = hardwareMap.servo.get("ring_knocker");

        //For the Wobble Goal
        wobbleClaw = hardwareMap.servo.get("wobble_claw");
        wobbleClaw2 = hardwareMap.servo.get("wobble_claw2");
        wobbleLifter = hardwareMap.servo.get("wobble_lifter");
        wobbleArm = hardwareMap.dcMotor.get("wobble_arm");
        wobbleTouch1 = hardwareMap.touchSensor.get("wobble_touch1");
        wobbleTouch2 = hardwareMap.touchSensor.get("wobble_touch2");

        //For the Flicker and Backplate
        backPlate = hardwareMap.servo.get("backplate");
        flicker = hardwareMap.servo.get("flicker");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights2 = hardwareMap.get(RevBlinkinLedDriver.class, "lights2");
    }

    public void initialize() {
        //Set motor directions
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set servo states
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);
        backPlate.setPosition(0);
        flicker.setPosition(0.23);

        ringKnocker.setPosition(0);
        wobbleLifter.setPosition(0.66);
        //wobbleLifter.setPosition(0.85);
    }

    public void followAsyncArm(Trajectory traj, double power) {
        drive.followTrajectoryAsync(traj);

        wobbleArm.setPower(power);

        while(drive.isBusy()) {
            boolean condition1 = wobbleTouch2.isPressed() && wobbleArm.getPower() > 0;
            boolean condition2 = wobbleTouch1.isPressed() && wobbleArm.getPower() < 0;
            if(condition1 || condition2) {
                wobbleArm.setPower(0);
            }

            drive.update();
        }
    }

    void Flick() {
        flickerStartTime = System.nanoTime();
        flicker.setPosition(0);
        while((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= 0.5 && opModeIsActive() && !isStopRequested()) {
            sleep(10);
        }
        flicker.setPosition(0.23);
        while((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= 0.75 && opModeIsActive() && !isStopRequested()) {
            sleep(10);
        }
    }

    public void putWobbleArmDown() {
        wobbleArm.setPower(-0.45);
    }

    public void putWobbleArmUp() {
        wobbleArm.setPower(0.37);
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

    public void releaseWobble() {
        wobbleClaw.setPosition(0);
        wobbleClaw2.setPosition(0);
    }

    public void grabWobble() {
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);
    }

    public void putWobbleLifterDown() {
        wobbleLifter.setPosition(0.85);
    }

    public void liftWobble() {
        wobbleLifter.setPosition(0.75);
    }

    public void putWobbleLifterUp() {
        wobbleLifter.setPosition(0.66);
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(75,115);

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
}
