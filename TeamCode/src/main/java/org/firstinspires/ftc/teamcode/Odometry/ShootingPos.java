package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Odometry.Tools.GlobalCoordinatePosition;

@Autonomous(name="Shooting Pos")
public class ShootingPos extends RobotMovement {

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
            goToPosition(15.8, 55.7, 0.5, 0, error, 0.5);
            turnToPosition(15.8, 100, 0.5);
            
            sleep(200000);
        }


        globalPositionUpdate.stop();

    }
}
