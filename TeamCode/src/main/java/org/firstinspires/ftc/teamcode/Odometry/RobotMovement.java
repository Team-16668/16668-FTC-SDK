package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Odometry.Tools.GlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Odometry.Tools.Line;
import org.firstinspires.ftc.teamcode.Odometry.Tools.MathFunctions;
import org.firstinspires.ftc.teamcode.Odometry.Tools.Point;

import java.io.File;
import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.util.Range.*;
import static java.lang.Math.*;

@Autonomous(name="Robot Movement")
public class RobotMovement extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //Ultimate Goal Specific Hardware
    DcMotor intake, wobbleArm;
    DcMotorEx shooter;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, ringKnocker, wobbleLifter;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;
    RevBlinkinLedDriver lights, lights2;

    //Logic for the Shooter
    double   normalTargetRPM = 4300,
            powerShotTargetRPM = 3500,
            shooterTargetRPM = normalTargetRPM;

    //Logic for the Flicker
    double flickerStartTime;

    //Background Task Variables
    boolean putWobbleArmDown = false;
    boolean putWobbleArmUp = false;
    boolean turnIntakeOnForward = false;
    boolean turnIntakeOnBackward = false;
    boolean stopIntake = false;
    boolean runShooterControl = false;
    boolean setPowerShotRPM = false;
    boolean setNormalRPM = false;
    boolean stopShooter = false;
    boolean setCustomRPM = false;


    final double COUNTS_PER_INCH = 307.699557;

    double distanceToTarget, robotX, robotY, robotOrientation, absoluteAngleToTarget, relativeAngleToTarget,
            relativeXToPoint, relativeYToPoint, movementXPower, movementYPower,
            movement_x, movement_y, relativeTurnAngle, movement_turn;
    double MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
    double leftFront, rightFront, rightBack, leftBack, lfAbs, rfAbs, rbAbs, lbAbs;

    //Hardware Map Names for drive motors and odometry wheels.
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    GlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        //Initialize hardware map values.
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions.
        globalPositionUpdate = new GlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        waitForStart();

        if(opModeIsActive()) {
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
        distanceToTarget = Math.hypot(x - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH), y - (-globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH));

        while (opModeIsActive() && !isStopRequested() && distanceToTarget > error) {
            robotX = globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x - (robotX), y - (robotY));

            absoluteAngleToTarget = Math.atan2(y - robotY, x - robotX);
            relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
            movementXPower = relativeXToPoint / MovementPowerDenominator;
            movementYPower = relativeYToPoint / MovementPowerDenominator;

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            relativeTurnAngle = relativeAngleToTarget - toRadians(90) + toRadians(preferredAngle);
            movement_turn = clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;

            if (distanceToTarget < 5) {
                movement_turn = 0;
            }

            telemetry.addData("absolute Angle", absoluteAngleToTarget);
            telemetry.addData("relative Angle", relativeAngleToTarget);
            telemetry.addData("relative turn Angle", relativeTurnAngle);

            telemetry.addData("movement_x", movement_x);
            telemetry.addData("movement_y", movement_y);
            telemetry.addData("movement_turn", movement_turn);

            telemetry.addData(" xpos", robotX);
            //telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation) + 90);
            telemetry.addData(" orientation", toDegrees(robotOrientation));
            telemetry.update();

            CalculateMotorPowers(movement_x, movement_y, movement_turn);

            BackGroundTaskCheck();
        }

        //Turns motors off
        StopMotors();

    }

    public void stayAtAbsoluteAngle(double x, double y, double movementSpeed, double preferredAbsoluteAngle, double error, double turnSpeed) {
        distanceToTarget = Math.hypot(x - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH), y - (-globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH));

        robotX = globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
        robotY = -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;

        Line line = calculateLineFromAngle(robotX, robotY, preferredAbsoluteAngle);

        while (opModeIsActive() && !isStopRequested() && distanceToTarget > error) {

            robotX = globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x - (robotX), y - (robotY));

            absoluteAngleToTarget = Math.atan2(y - robotY, x - robotX);
            relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
            movementXPower = relativeXToPoint / MovementPowerDenominator;
            movementYPower = relativeYToPoint / MovementPowerDenominator;

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            Point currentPoint = new Point(robotX, robotY);

            Line currentLine = perpendicularThroughPoint(line, currentPoint);

            Point turnToPoint = Intersection(line, currentLine);

            double turnToAbsoluteAngleToTarget = Math.atan2(turnToPoint.y - robotY, turnToPoint.x - robotX);
            double turnToRelativeAngleToTarget = MathFunctions.AngleWrap(turnToAbsoluteAngleToTarget - (robotOrientation));

            double turnToRelativeTurnAngle = turnToRelativeAngleToTarget - toRadians(90);
            movement_turn = clip(turnToRelativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;

            if (distanceToTarget < 5) {
                movement_turn = 0;
            }

            telemetry.addData("Target Point X", turnToPoint.x);
            telemetry.addData("Target Point Y", turnToPoint.y);

            telemetry.addData("movement_x", movement_x);
            telemetry.addData("movement_y", movement_y);
            telemetry.addData("movement_turn", movement_turn);

            telemetry.addData(" xpos", robotX);
            //telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation) + 90);
            telemetry.addData(" orientation", toDegrees(robotOrientation));
            telemetry.update();

            CalculateMotorPowers(movement_x, movement_y, movement_turn);

            BackGroundTaskCheck();
        }

        //Turns motors off
        StopMotors();

    }

    public Line calculateLineFromAngle(double robotPosX, double robotPosY, double orientation) {
        Point startingPoint = new Point(robotPosX, robotPosY);

        Point targetPoint = castPoint(orientation, startingPoint);

        Line line = LineFromTwoPoints(startingPoint, targetPoint);

        return perpendicularThroughPoint(line, targetPoint);
    }

    public Point castPoint(double orientation, Point startingPoint) {
        Point targetPoint = new Point(0, 0);
        double xComponent = 0;
        double yComponent = 0;

        double angle = orientation;
        if(angle==0) {
            angle=1;
        } else if (angle == 90) {
            angle=91;
        } else if (angle == 180) {
            angle = 179;
        } else if (angle == -90) {
            angle = -91;
        }
        if(angle >= 0 && angle <= 90) {
            xComponent = 1000 * Math.sin(angle);
            yComponent = 1000 * Math.cos(angle);

            targetPoint.x = xComponent + startingPoint.x;
            targetPoint.y = yComponent + startingPoint.y;
        } else if(angle >=90 && angle <= 180) {
            angle -= 90;

            xComponent = 1000 * Math.cos(angle);
            yComponent = 1000 * Math.sin(angle);

            targetPoint.x = xComponent + startingPoint.x;
            targetPoint.y = yComponent + startingPoint.y;
        }else if (angle <= 0 && angle >= -90) {
            angle *= -1;

            xComponent = 1000 * Math.sin(angle);
            yComponent = 1000 * Math.cos(angle);

            targetPoint.x = -xComponent + startingPoint.x;
            targetPoint.y = -yComponent + startingPoint.y;
        }else if (angle <= -90 && angle >= -180) {
            angle += 90;

            xComponent = 1000 * Math.cos(angle);
            yComponent = 1000 * Math.sin(angle);

            targetPoint.x = -xComponent + startingPoint.x;
            targetPoint.y = -yComponent + startingPoint.y;
        }

        //Point startingPoint = new Point(robotPosX, robotPosY);
        return targetPoint;
    }

    public Point Intersection(Line line1, Line line2) {
        double xIntersection = (line2.b - line1.b) / (line1.m-line2.m);
        double yIntersection = (line1.m*xIntersection) + line1.b;

        return new Point(xIntersection, yIntersection);
    }



    /**
     *
     * @param x x coordinate to turn and go to
     * @param y y coordinate to turn and go to
     * @param movementSpeed movement speed
     * @param preferredAngle preferred angle (0 = straight, 90 = to the right, -90 = to the left, use 179 or -179 for backwards, as 180 wraps weirdly)
     * @param error how close (in inches) you need to get to the point. The closer you get, the higher the chance of spasing (spelling?) around the point
     * @param turnSpeed1 how fast the robot should turn towards the position
     * @param turnSpeed2 how fast the robot should turn when it's moving tp tje [pomt
     */
    public void turnAndGo(double x, double y, double movementSpeed, double preferredAngle, double error, double turnSpeed1, double turnSpeed2) {
        turnToPosition(x, y, turnSpeed1, preferredAngle,.2, 30);
        goToPosition(x, y, movementSpeed, preferredAngle, error, turnSpeed2);
    }

    public void goToPositionWithoutTurn(double x, double y, double movementSpeed, double error) {
        distanceToTarget = Math.hypot(x-(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH), y-(-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));

        while(opModeIsActive() && !isStopRequested() && distanceToTarget > error) {
            robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x-(robotX), y-(robotY));

            absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
            movementXPower = relativeXToPoint / MovementPowerDenominator;
            movementYPower = relativeYToPoint / MovementPowerDenominator;

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            telemetry.addData(" xpos", robotX);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", toDegrees(robotOrientation)+90);
            telemetry.update();

            movement_turn = 0;

            CalculateMotorPowers(movement_x, movement_y, movement_turn);

            BackGroundTaskCheck();
        }

        //Turns motors off
        StopMotors();

    }

    public void turnToPosition(double x, double y, double preferredAngle, double turnSpeed, double threshhold, double scale) {

        robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
        robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

        absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);

        relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));
        relativeTurnAngle = relativeAngleToTarget - toRadians(90) + toRadians(preferredAngle);

        movement_turn = clip(relativeTurnAngle / toRadians(scale), -1, 1) * turnSpeed;

        while(opModeIsActive() && !isStopRequested() && abs(movement_turn) > threshhold) {
            telemetry.addData("loop finished", abs(movement_turn) > threshhold);
            telemetry.addData("moving", abs(movement_turn)-threshhold);
            telemetry.addData("threshold", threshhold);
            robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            relativeTurnAngle = relativeAngleToTarget - toRadians(90) + toRadians(preferredAngle);
            movement_turn = clip(relativeTurnAngle / toRadians(scale), -1, 1) * turnSpeed;

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

            BackGroundTaskCheck();
        }

        //Turns motors off
        StopMotors();

    }

    public void turnToPositionNoStop(double x, double y, double turnSpeed) {

        robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
        robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

        absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);

        relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));
        relativeTurnAngle = relativeAngleToTarget - toRadians(90);

        movement_turn = clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;

        while(opModeIsActive() && !isStopRequested() && abs(movement_turn) > 0.25) {
            robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            relativeAngleToTarget = MathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

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

            BackGroundTaskCheck();
        }

    }

    public void BackGroundTaskCheck() {
        if(putWobbleArmDown) {
            wobbleArm.setPower(-0.45);
            MoveWobbleArm();
        }
        if(putWobbleArmUp) {
            wobbleArm.setPower(0.37);
            MoveWobbleArm();
        }
        if(turnIntakeOnForward) {
            intake.setPower(1);
            intakeServo.setPower(1);
            turnIntakeOnForward = false;
        }
        if(turnIntakeOnBackward) {
            intake.setPower(-1);
            turnIntakeOnBackward = false;
        } if(stopIntake) {
            intakeServo.setPower(0);
            intake.setPower(0);
            stopIntake = false;
        }
        if(runShooterControl) {
            shooter.setVelocity((shooterTargetRPM*28)/60);
            telemetry.addData("Shooter Speed", (shooter.getVelocity()/28)*60);
            runShooterControl = false;
        }
        if(stopShooter) {
            shooter.setVelocity(0);
            stopShooter = false;
        }
        if(setPowerShotRPM) {
            shooterTargetRPM = powerShotTargetRPM;
            runShooterControl = true;
            setPowerShotRPM = false;
        }
        if(setNormalRPM) {
            shooterTargetRPM = normalTargetRPM;
            runShooterControl = true;
            setNormalRPM = false;
        }
        if(setCustomRPM) {
            shooterTargetRPM = 4100;
            runShooterControl = true;
            setCustomRPM = false;
        }
    }

    private void CalculateMotorPowers(double movement_x, double movement_y, double movement_turn) {
        leftFront = -movement_y - movement_x + movement_turn;
        rightFront = movement_y - movement_x + movement_turn;
        rightBack = movement_y + movement_x + movement_turn;
        leftBack = -movement_y + movement_x + movement_turn;

        //Scale Motor Powers to preserve movement shape
        lfAbs = abs(leftFront);
        rfAbs = abs(rightFront);
        rbAbs = abs(rightBack);
        lbAbs = abs(leftBack);

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

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public void StopMotors() {
        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
    }

    public double interpretAngle(double Orientation) {
        if(isWithin(Orientation, -180, 0)) {
            return Orientation;
        } else if (isWithin(Orientation, -360, -180)) {
            return (Orientation +360);
        } else {
            return Orientation;
        }
    }

    public boolean isWithin(double value, double min, double max) {
        if(value >= max) {
            return false;
        } else if (value <= min) {
            return false;
        } else {
            return true;
        }
    }

    public Line LineFromTwoPoints(Point p1, Point p2) {
        double slope = (p2.y-p1.y)/(p2.x-p1.y);
        return new Line(slope, p1.y - (slope * p1.x));
    }

    public Line perpendicularThroughPoint(Line line, Point point) {
        double m = -(1/line.m);
        double b = point.y - (m*point.x);

        return new Line(m, b);
    }

    public void MoveWobbleArm() {
        boolean condition1 = wobbleTouch2.isPressed() && wobbleArm.getPower() > 0;
        boolean condition2 = wobbleTouch1.isPressed() && wobbleArm.getPower() < 0;
        if(condition1 || condition2) {
            wobbleArm.setPower(0);
            putWobbleArmDown = false;
            putWobbleArmUp = false;
        }
    }

    //Discrete Functions I need while Not Moving
    void Flick() {
        flickerStartTime = System.nanoTime();
        flicker.setPosition(0);
        while((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= 0.5 && opModeIsActive() && !isStopRequested()) {
            sleep(10);
        }
        flicker.setPosition(1);
        while((System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1) <= 1 && opModeIsActive() && !isStopRequested()) {
            sleep(10);
        }
    }
}