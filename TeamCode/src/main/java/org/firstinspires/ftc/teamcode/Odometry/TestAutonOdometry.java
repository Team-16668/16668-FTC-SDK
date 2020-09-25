package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Odometry Autonomous")
public class TestAutonOdometry extends RobotMovement {

   @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

       telemetry.addData(" Status", " Waiting for Start");
       telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            turnAndGo(-1.48, 111.15, 0.5, 0, 2, 0.3, 0.3);
            turnToPositionNoStop(-61.49, 150, 0.3);
            turnAndGo(-61.49, 112.33, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(11.76, 116.87, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(14.55, 49.18, 0.5, 0, 2, 0.3, 0.3);

            turnToPosition(14.55, 38.18, 0.3);

            sleep(500);
        }


        globalPositionUpdate.stop();

    }
}
