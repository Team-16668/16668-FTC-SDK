package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.*;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Odometry.mathFunctions.interpretAngle;

//test
@Autonomous(name="Circle")
public class CircleMovement extends RobotMovement {

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
            circle(50, 0.25, 0, 0.5, 0.25);
            sleep(500);
        }


        globalPositionUpdate.stop();

    }

    public void circle(double radius, double movementSpeed, double preferredAngle, double error, double turnSpeed) {

        double originalX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        double originalY = globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;

        double robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        double robotY = globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;

        turnToPosition(robotX, robotY+1, turnSpeed);

        goToPosition(sin(0)*radius, cos(0)*radius, movementSpeed, preferredAngle, error, turnSpeed);

        turnToPosition(originalX+radius, robotY, turnSpeed);

        final Point[] points = new Point[360];

        for (int i=0; i <= 360; i++) {
            final double angle = toRadians(((double) i/360)*360d);
            goToPosition(sin(angle)*radius, cos(angle)*radius, movementSpeed, preferredAngle, error, turnSpeed);
        }

        turnAndGo(originalX, originalY, movementSpeed, preferredAngle, error, turnSpeed, turnSpeed);
        
        turnToPosition(robotX, robotY+10, turnSpeed);
    }

    @Override
    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double error, double turnSpeed) {
        double distanceToTarget = Math.hypot(x-(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH), y-(-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));

        while(opModeIsActive() && distanceToTarget > error) {
            double robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            double robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            double robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x-(robotX), y-(robotY));

            double absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            double relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            double MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
            double movementXPower = relativeXToPoint / MovementPowerDenominator;
            double movementYPower = relativeYToPoint / MovementPowerDenominator;

            double movement_x = movementXPower * movementSpeed;
            double movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToTarget - toRadians(90) + toRadians(preferredAngle);
            double movement_turn = clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;
            telemetry.addData(" xpos", robotX);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation)+90);
            telemetry.update();

            double leftFront = -movement_y - movement_x + movement_turn;
            double rightFront = movement_y - movement_x + movement_turn;
            double rightBack = movement_y + movement_x + movement_turn;
            double leftBack = -movement_y + movement_x + movement_turn;

            //Scale Motor Powers to preserve movement shape
            double lfAbs = abs(leftFront);
            double rfAbs = abs(rightFront);
            double rbAbs = abs(rightBack);
            double lbAbs = abs(leftBack);

            if(lfAbs > rfAbs && lfAbs > rbAbs && lfAbs > lbAbs && lfAbs > 1) {
                leftFront = leftFront/lfAbs;
                rightFront = rightFront/lfAbs;
                rightBack = rightBack/lfAbs;
                leftBack = leftBack/lfAbs;
            } else if(rfAbs > lfAbs && rfAbs > rbAbs && rfAbs > lbAbs && rfAbs > 1) {
                leftFront = leftFront/rfAbs;
                rightFront = rightFront/rfAbs;
                rightBack = rightBack/rfAbs;
                leftBack = leftBack/rfAbs;
            } else if(rbAbs > lfAbs && rbAbs > rfAbs && rbAbs > lbAbs && rbAbs > 1) {
                leftFront = leftFront/rbAbs;
                rightFront = rightFront/rbAbs;
                rightBack = rightBack/rbAbs;
                leftBack = leftBack/rbAbs;
            } else if(lbAbs > lfAbs && lbAbs > rfAbs && lbAbs > rbAbs && lbAbs > 1) {
                leftFront = leftFront/lbAbs;
                rightFront = rightFront/lbAbs;
                rightBack = rightBack/lbAbs;
                leftBack = leftBack/lbAbs;
            }

            leftFront = clip(leftFront, -1, 1);
            rightFront = clip(rightFront, -1, 1);
            rightBack = clip(rightBack, -1, 1);
            leftBack = clip(leftBack, -1, 1);

            right_front.setPower(rightFront);
            right_back.setPower(rightBack);
            left_front.setPower(leftFront);
            left_back.setPower(leftBack);
        }

        //Turns motors off
        StopMotors();

    }
}
