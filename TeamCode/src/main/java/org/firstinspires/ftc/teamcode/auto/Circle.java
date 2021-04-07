package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class Circle extends LinearOpMode {

    //Ultimate Goal Specific Hardware
    DcMotor intake, wobbleArm;
    DcMotorEx shooter;
    Servo wobbleClaw, wobbleClaw2, backPlate, flicker, ringKnocker, wobbleLifter;
    CRServo intakeServo;
    TouchSensor wobbleTouch1, wobbleTouch2;
    RevBlinkinLedDriver lights, lights2;

    SampleMecanumDrive drive;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        timer = new ElapsedTime();
        //Wonderful timer commands that are probably a lot better than what I have now.
        timer.reset();
        timer.seconds();

        defineHardware();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(24, 0), 0)
                .splineToConstantHeading(new Vector2d(24, -24), 0)
                .splineToConstantHeading(new Vector2d(0, 0), 0)
                .splineToConstantHeading(new Vector2d(0, 24), 0)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
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

    public void followAsyncArm(Trajectory traj) {
        drive.followTrajectoryAsync(traj);

        while(drive.isBusy()) {
            boolean condition1 = wobbleTouch2.isPressed() && wobbleArm.getPower() > 0;
            boolean condition2 = wobbleTouch1.isPressed() && wobbleArm.getPower() < 0;
            if(condition1 || condition2) {
                wobbleArm.setPower(0);
            }

            drive.update();
        }
    }

    public void putWobbleArmDown() {
        wobbleArm.setPower(-0.45);
    }

    public void putWobbleArmUp() {
        wobbleArm.setPower(0.37);
    }

    public void setShooterRPM(double RPM) {
        shooter.setVelocity((RPM/60)*28);
    }

    public void runIntakeForward() {
        intake.setPower(1);
        intakeServo.setPower(1);
    }

    public void runIntakeBackward() {
        intake.setPower(-1);
        intakeServo.setPower(-1);
    }

    public void stopIntake() {
        intake.setPower(0);
        intakeServo.setPower(0);
    }

    public void liftBackplate() {
        backPlate.setPosition(0);
    }

    public void lowerBackplate() {
        backPlate.setPosition(1);
    }

    public void releaseWobble() {
        wobbleClaw.setPosition(0);
        wobbleClaw2.setPosition(0);
    }

    public void grabWobble() {
        wobbleClaw.setPosition(1);
        wobbleClaw2.setPosition(1);
    }

    public void putWobbleLifterDown() {
        wobbleLifter.setPosition(0.85);
    }

    public void liftWobble() {
        wobbleLifter.setPosition(0.75);
    }

    public void putWobbleLifterUp() {
        wobbleLifter.setPosition(0.66);
    }

    public void putDownRingKnocker() {
        ringKnocker.setPosition(1);
    }

    public void liftUpRingKnocker() {
        ringKnocker.setPosition(0);
    }

    public void setLightPattern(int lightSet, RevBlinkinLedDriver.BlinkinPattern pattern) {
        if(lightSet == 1) {
            lights.setPattern(pattern);
        } else if (lightSet == 2) {
            lights2.setPattern(pattern);
        }
    }
}
