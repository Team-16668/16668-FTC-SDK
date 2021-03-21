package org.firstinspires.ftc.teamcode.RandomTools.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RandomTools.Odometry.Tools.GlobalCoordinatePosition;

@Disabled
@Autonomous(name="Absolute Angle Test")
public class AbsoluteAngleTest extends RobotMovement {

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
            stayAtAbsoluteAngle(0, 100, 0.5, 45, 2, 0.5);

            sleep(200000);
        }


        globalPositionUpdate.stop();

    }
}
