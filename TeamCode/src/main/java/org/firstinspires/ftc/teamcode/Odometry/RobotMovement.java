package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.util.Range.*;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.Odometry.mathFunctions.interpretAngle;

@Autonomous(name="Robot Movement")
public class RobotMovement extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels.
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    globalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        //Initialize hardware map values.
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions.
        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        waitForStart();

        if(opModeIsActive()) {
            turnAndGo(0, 12, 0.5, 0, 2, 0.3, 0.3);
            turnToPositionNoStop(-50,62, 0.3);
            turnAndGo(-60, 12, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(-90, 54, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(-60, 110, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(0, 110, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(13, 70, 0.5, 0, 2, 0.3, 0.3);
            turnAndGo(0, 10, 0.5, 0, 2, 0.3, 0.3);
            turnToPosition(0, 12, 0.3);
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    //This function handles the actual movement of the robot.

    /**
     * @param x X point to go to
     * @param y Y point to go to
     * @param movementSpeed Maximum linear speed
     * @param preferredAngle Angle that you want the robot to move toward the point at
     * @param error How close to the point you want the robot to get (in inches)
     * @param turnSpeed How fast the robot can turn
     */

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

            if(distanceToTarget < 5) {
                movement_turn = 0;
            }

            telemetry.addData(" xpos", robotX);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation)+90);
            telemetry.update();

            //Testing values
            //movement_x = 0;
            //movement_y = 0;
            //movement_turn = 0;

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

    public void turnAndGo(double x, double y, double movementSpeed, double preferredAngle, double error, double turnSpeed1, double turnSpeed2) {
        turnToPosition(x, y, turnSpeed1);
        goToPosition(x, y, movementSpeed, preferredAngle, error, turnSpeed2);
    }

    public void goToPositionWithoutTurn(double x, double y, double movementSpeed, double preferredAngle, double error) {
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

            //Lots of Telemetry. Remove some of this.

            //telemetry.addData( " x" , movement_x);
            //telemetry.addData(" y", movement_y);
            //telemetry.addData(" theta", movement_turn);
            //telemetry.addData( " Distance to Target", distanceToTarget);
            //telemetry.addData( " Absolute Angle to Target", Math.toDegrees(absoluteAngleToTarget));
            //telemetry.addData( " Relative to Target", Math.toDegrees(relativeAngleToTarget));
            //telemetry.addData( " Relative Turn Angle", Math.toDegrees(relativeTurnAngle));
            telemetry.addData(" xpos", robotX);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation)+90);
            telemetry.update();

            //Testing values
            //movement_x = 0;
            //movement_y = 0;
            double movement_turn = 0;

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

    public void turnToPosition(double x, double y, double turnSpeed) {

        double robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        double robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
        double robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

        double absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);

        double relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));
        double relativeTurnAngle = relativeAngleToTarget - toRadians(90);

        double movement_turn = clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;

        while(opModeIsActive() && abs(movement_turn) > 0.1) {
            robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeTurnAngle = relativeAngleToTarget - toRadians(90);
            movement_turn = clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;

            telemetry.addData(" xpos", robotX);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation)+90);
            telemetry.addData(" movement_turn", abs(movement_turn));
            telemetry.update();

            right_front.setPower(clip(movement_turn, -1, 1));
            right_back.setPower(clip(movement_turn, -1, 1));
            left_front.setPower(clip(movement_turn, -1, 1));
            left_back.setPower(clip(movement_turn, -1, 1));
        }

        //Turns motors off
        StopMotors();

    }

    public void turnToPositionNoStop(double x, double y, double turnSpeed) {

        double robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        double robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
        double robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

        double absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);

        double relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));
        double relativeTurnAngle = relativeAngleToTarget - toRadians(90);

        double movement_turn = clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;

        while(opModeIsActive() && abs(movement_turn) > 0.25) {
            robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeTurnAngle = relativeAngleToTarget - toRadians(90);
            movement_turn = clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;

            telemetry.addData(" xpos", robotX);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation)+90);
            telemetry.addData(" movement_turn", abs(movement_turn));
            telemetry.update();

            right_front.setPower(clip(movement_turn, -1, 1));
            right_back.setPower(clip(movement_turn, -1, 1));
            left_front.setPower(clip(movement_turn, -1, 1));
            left_back.setPower(clip(movement_turn, -1, 1));
        }

    }

    //Function to initialize all the motors and encoders. Used when preparing for the OpMode to start.
    void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public void StopMotors() {
        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
    }
}
