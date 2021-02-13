package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

import static java.lang.Math.toRadians;

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

        initHardwareMap();

        telemetry.addData(" Status", " Waiting for Start");
        telemetry.update();

        wobbleClaw.setPosition(1);
        backPlate.setPosition(0);
        flicker.setPosition(1);

        waitForStart();

        double error = 2;

        if(opModeIsActive()) {

            setPowerShotRPM = true;
            runShooterControl = true;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                //Zero Rings
                webcam.stopStreaming();
            } else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                //One Ring
                webcam.stopStreaming();
            } else if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.FOUR) {
                //Four Rings
                webcam.stopStreaming();
            }

            goToPosition(0, 15, 1, 0, 5, 0.5);
            goToPosition(-17, 49, 1, 0, 2, 0.5);
            turnAndGo(-17, 59, 0.25, 0, 0.25, 0.25, 0.2);
            //Shoot Powershots here
            turnToPosition(-18, 100,0, 0.25, 0.15, 30);
            Flick();
            goToPosition(-11, 59,0.25, 90, 0.15, .25);
            turnToPosition(-12, 62, 0, 0.25, 0.15, 30);
            Flick();
            goToPosition(-4, 59,0.25, 90, 0.15, .25);
            turnToPosition(-5, 62, 0, 0.25, 0.15, 30);
            Flick();

            stopShooter = true;

            putWobbleArmDown = true;

            if(pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.NONE) {
                turnToPosition(-16, 75, 0, 0.3, 0.2, 30);
                goToPosition(20, 77, 0.5, 179, 3, 0.3);
                goToPosition(25, 79, 0.25, 179, 2, 0.3);
                turnToPosition(30, 79, 179, 0.3, 0.2, 30);
                //Release 1st Wobble
                wobbleClaw.setPosition(0);
                sleep(500);
                putWobbleArmUp = true;

                goToPosition(20, 81, 0.25, 0, 2, 0.3);
                wobbleClaw.setPosition(1);
                goToPosition(17, 24, 1, 0, 5, 0.3);
                //Pick up 2nd Wobble here
                turnToPosition(21, 60, 0, 0.3, 0.2, 30);
                goToPosition(21, 60, 0.5, 0, 1, 0.3);
                goToPosition(12, 51, 0.25, 179, 1, 0.3);
                //Release 2nd Wobble here
                goToPosition(5, 67, 1, 179, 2, 0.3);

            } else if (pipeline.position == OpenCVWebcam.SkystoneDeterminationPipeline.RingPosition.ONE) {
                turnToPosition(-25, 62, 0, 0.3, 0.2, 30);
                goToPosition(-5, 92, 0.5, 179, 3, 0.3);
                goToPosition(0, 92, 0.25, 179, 2, 0.3);
                turnToPosition(30, 79, 179, 0.3, 0.2, 30);
                //Release 1st Wobble
                wobbleClaw.setPosition(0);
                sleep(500);
                putWobbleArmUp = true;

                goToPosition(20, 81, 0.25, 0, 2, 0.3);
                wobbleClaw.setPosition(1);
                goToPosition(17, 24, 1, 0, 5, 0.3);
                //Pick up 2nd Wobble here
                turnToPosition(21, 60, 0, 0.3, 0.2, 30);
                goToPosition(21, 60, 0.5, 0, 1, 0.3);
                goToPosition(12, 51, 0.25, 179, 1, 0.3);
                //Release 2nd Wobble here
                goToPosition(5, 67, 1, 179, 2, 0.3);
            }


            ReadWriteFile.writeFile(xPosFile, String.valueOf(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH));
            ReadWriteFile.writeFile(yPosFile, String.valueOf(-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));
            ReadWriteFile.writeFile(orientationFile, String.valueOf(interpretAngle(globalPositionUpdate.returnOrientation())));

            sleep(200000);
        }


        globalPositionUpdate.stop();

    }

    public void initHardwareMap() {


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