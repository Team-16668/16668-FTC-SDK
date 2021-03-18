package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Odometry.Tools.GlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Vision.OpenCVWebcam;
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

import java.io.File;

@Autonomous(name="Auton")
public class Auton extends RobotMovement {

    OpenCvWebcam webcam;
    SkystoneDeterminationPipeline pipeline;

    File xPosFile = AppUtil.getInstance().getSettingsFile("xPos.txt");
    File yPosFile = AppUtil.getInstance().getSettingsFile("yPos.txt");
    File orientationFile = AppUtil.getInstance().getSettingsFile("orientation.txt");

   @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        globalPositionUpdate = new GlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

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

        telemetry.addData(" Status", " Waiting for Start");
        telemetry.update();

        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);
        backPlate.setPosition(0);
        flicker.setPosition(1);

        ringKnocker.setPosition(0);
        wobbleLifter.setPosition(0.66);
        //wobbleLifter.setPosition(0.8);

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

        double error = 2;

        if(opModeIsActive()) {

            ringKnocker.setPosition(1);
            sleep(1000);

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

            InitialShots();

            if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                goToPosition(10, 40, 0.5, 0, 5, 0.3);
                ringKnocker.setPosition(0);
                putWobbleArmDown = true;
                turnToPosition(-16, 56, 0, 1, 0.2, 30);
                goToPosition(20, 75, 0.5, 179, 6, 0.3);
                goToPosition(26, 77, 0.25, 179, 2, 0.3);
                turnToPosition(30, 79, 179, 0.3, 0.2, 30);

                //Release 1st Wobble
                wobbleClaw.setPosition(0);
                wobbleClaw2.setPosition(0);
                sleep(500);

                GetAwayFromWobble1();
                //goToPosition(21, 80, 0.5, 179, 2, 0.3);

                //Get the second wobble
                GetSecondWobble();

                turnToPositionNoStop(100, 100, 0.6);

                turnToPosition(10, 70, 0, 0.3, 0.2, 30);
                goToPosition(13, 70, 0.5, 0, 2, 0.3);
                turnToPosition(13, 79, 0, 0.3, 0.15, 30);
                wobbleLifter.setPosition(0.89);
                sleep(750);

                StrafeFromWobble();

                goToPosition(-10, 67, 1, 179, 2, 0.3);

            } else if (pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                //setNormalRPM = true;
                shooterTargetRPM = 4400;

                turnIntakeOnForward = true;
                goToPosition( 10, 40, 0.5, 0, 2, 0.3);
                putWobbleArmDown = true;
                turnAndGo(11, 54, 0.5,0, 5, 0.25, 0.2);

                ringKnocker.setPosition(0);
                //This is what you get rid of to take out the shot
                sleep(1000);
                backPlate.setPosition(0);
                sleep(1000);
                stopIntake = true;

                turnAndGo(11, 54, 0.25, 0, 0.5, 0.25, 0.2);
                //turnToPosition(10, 100, 0, 0.25, 0.15, 30);

                Flick();

                //This is where getting rid of ends

                stopShooter = true;

                turnToPosition(0, 0, 0, 1, 0.2, 30);
                goToPosition(10, 80, 1, 179, 10, 0.3);
                goToPosition(9, 84, 0.25, 179, 2, 0.3);
                //Release 1st Wobble
                wobbleClaw.setPosition(0);
                wobbleClaw2.setPosition(0);
                sleep(750);

                GetAwayFromWobble1();

                //goToPosition(10, 76, 0.3, 179, 2, 0.3);
                wobbleClaw.setPosition(1);
                wobbleClaw2.setPosition(1);

                GetSecondWobble();

                //Turn Towards the second wobble position.
                turnToPositionNoStop(100, 100, 0.5);
                turnToPosition(-4, 100, 0, 0.5, 0.3, 30);

                goToPosition(-9, 85, 1,0, 10, 0.5);
                goToPosition(-9, 87, 0.25,0, 2, 0.5);
                turnToPosition(-10, 115, 0, 0.5, 0.15,15);
                wobbleLifter.setPosition(0.9);
                sleep(500);

                StrafeFromWobble();

                wobbleLifter.setPosition(0.65);
                goToPosition(-20, 75, 0.75, 179, 4, 0.3);
            } else {
                //shooter.setVelocity((4400*28)/60);
                shooterTargetRPM = 4400;

                turnIntakeOnForward = true;
                goToPositionWithoutTurn(9, 32, 0.3, 2);
                sleep(2000);
                backPlate.setPosition(0);
                sleep(500);
                Flick();
                backPlate.setPosition(1);

                goToPosition(10, 64, 0.25, 0, 5, 0.3);
                ringKnocker.setPosition(0);
                stopShooter = true;

                putWobbleArmDown = true;
                stopIntake = true;
                turnToPosition(0, 0, 0, 1, 0.25, 30);

                goToPosition(26, 95, 1, 179, 5, 0.3);
                goToPosition(30, 110, 0.25, 179, 1, 0.3);

                wobbleClaw.setPosition(0);
                wobbleClaw2.setPosition(0);

                sleep(750);

                GetAwayFromWobble1();

                wobbleClaw.setPosition(1);
                wobbleClaw2.setPosition(1);

                //This is where I get the second wobble. There appears to be some error in the odometry wheels, which makes this one differet.

                turnToPosition(19, 36, 90, 0.75, 0.3, 30);
                wobbleLifter.setPosition(0.85);
                goToPosition(22, 45, 1, 90, 5, 0.5);
                //Get rid of this line if needed
                goToPosition(22, 40, 0.25, 90, 5, 0.5);
                turnToPosition(22.52, 0, 90, 0.3, 0.15, 30);
                goToPosition(23, 22, 0.45, 90, 1, 0.1);
                wobbleLifter.setPosition(0.75);
                sleep(500);

                turnToPositionNoStop(100, 100, 1);

                goToPosition(12, 95, 1, 0, 5, 0.5);
                goToPosition(12, 114, 0.3, 0, 1, 0.3);

                wobbleLifter.setPosition(0.89);
                sleep(750);

                StrafeFromWobble();

                goToPosition(10, 80, 1, 179, 5, 0.3);
            }

            StopMotors();


            ReadWriteFile.writeFile(xPosFile, String.valueOf(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH));
            ReadWriteFile.writeFile(yPosFile, String.valueOf(-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));
            ReadWriteFile.writeFile(orientationFile, String.valueOf(interpretAngle(globalPositionUpdate.returnOrientation())));
        }


        globalPositionUpdate.stop();

    }

    private void GetAwayFromWobble1() {
        right_front.setPower(1);
        right_back.setPower(1);
        left_front.setPower(-1);
        left_back.setPower(-1);
        sleep(250);
    }

    private void StrafeFromWobble() {
        right_front.setPower(1);
        left_front.setPower(1);
        right_back.setPower(-1);
        left_back.setPower(-1);
        sleep(500);
    }

    private void GetSecondWobble() {
       turnToPosition(17.5, 36, 90, 0.75, 0.3, 30);
       wobbleLifter.setPosition(0.85);
       goToPosition(18, 45, 1, 90, 5, 0.5);
       //Get rid of this line if needed
       goToPosition(18, 40, 0.25, 90, 5, 0.5);
       turnToPosition(19.25, 0, 90, 0.3, 0.15, 30);
       goToPosition(18.5, 25, 0.45, 90, 1, 0.1);
       wobbleLifter.setPosition(0.75);
       sleep(500);
    }

    private void InitialShots() {
        setCustomRPM = true;
        runShooterControl = true;

        goToPositionWithoutTurn(12, 21, 0.5, 2);
        turnToPosition(30, 75, 0, 0.25, 0.15, 30);
        Flick();
        Flick();
        Flick();
        sleep(1000);
        backPlate.setPosition(1);
        sleep(500);
    }

    public void PowerShotCode() {
        turnAndGo(14, 56, 0.125, 0, 0.25, 0.25, 0.2);
        //Shoot Powershots here
        turnToPosition(-10, 100,0, 0.25, 0.15, 30);
        Flick();
        turnToPosition(-15, 100, 0, 0.25, 0.15, 30);
        Flick();
        setNormalRPM = true;
        turnToPosition(-18, 100, 0, 0.25, 0.15, 30);
        Flick();

        stopShooter = true;
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
