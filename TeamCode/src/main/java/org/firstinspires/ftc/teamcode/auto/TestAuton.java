package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RandomTools.Odometry.Tools.GlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class TestAuton extends LinearOpMode {

    //Ultimate Goal Specific Hardware
    DcMotor intake, wobbleArm;
    DcMotorEx shooter;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, ringKnocker, wobbleLifter;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;
    RevBlinkinLedDriver lights, lights2;

    BackgroundThread background;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));

        defineHardware();

        //Create and start background thread to constantly update the robot's background tasks.
        background = new BackgroundThread(shooter, intake, wobbleArm, wobbleClaw, wobbleClaw2,
                backPlate, flicker, ringKnocker, wobbleLifter, intakeServo, wobbleTouch1, wobbleTouch2,
                lights, lights2, 75);
        Thread backgroundThread = new Thread(background);
        backgroundThread.start();

        background.init();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-62, -26, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-37, -35.5), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(45, -40, Math.toRadians(135)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-35, -42, Math.toRadians(270)), 0)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(57, -45, Math.toRadians(0)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToLinearHeading(new Pose2d(8, -38, Math.toRadians(0)), 0)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        Thread.sleep(2000);
        drive.followTrajectory(traj2);
        Thread.sleep(2000);
        drive.followTrajectory(traj3);
        Thread.sleep(2000);
        drive.followTrajectory(traj4);
        Thread.sleep(2000);
        drive.followTrajectory(traj5);
        Thread.sleep(2000);
    }

    private void defineHardware() {
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
    }
}
