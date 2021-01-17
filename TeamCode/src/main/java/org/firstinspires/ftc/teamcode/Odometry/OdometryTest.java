package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="New Odometry Test")
public class OdometryTest extends RobotMovement {

   @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        globalPositionUpdate = new GlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

       telemetry.addData(" Status", " Waiting for Start");
       telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            goToPosition(12, 0, 0.5, 0, 2, 0.5);
            /*
            turnAndGo(12, 0, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(0, 12, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(-12, 0, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(0, -12, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(0, 0, 0.5, 0, 2, 0.5, 0.3);
             */

            sleep(500);
        }


        globalPositionUpdate.stop();

    }
}
