package org.firstinspires.ftc.teamcode.RandomTools.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RandomTools.Odometry.Tools.GlobalCoordinatePosition;

@Disabled
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

        double error = 2;

        if(opModeIsActive()) {
            turnToPosition(0,24, 179, 0.3, 0.2, 30);
            //goToPosition(0, -24, 0.5, 179, 2, 0.3);
            /*
            goToPosition(0, 10, 0.5, 0, error, 0.5);
            turnAndGo(7, 95.5, 0.5, 0, error, 0.5, 0.5);
            turnAndGo(27, 73, 0.5, 0, error, 0.5, 0.5);
            turnToPositionNoStop(1000, 1000, 0.5);
            turnAndGo(29, 116, 0.5, 0, error, 0.5, 0.5);
            turnAndGo(0, 10, 0.5, 0, error, 0.5, 0.5);
               */
            /*
            turnAndGo(12, 0, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(0, 12, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(-12, 0, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(0, -12, 0.5, 0, 2, 0.5, 0.3);
            turnAndGo(0, 0, 0.5, 0, 2, 0.5, 0.3);
             */

            sleep(200000);
        }


        globalPositionUpdate.stop();

    }
}
