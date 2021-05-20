package org.firstinspires.ftc.teamcode.tools;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@TeleOp(name="reset position")
public class ResetPos extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Position reset", "waiting for start");
        telemetry.update();

        waitForStart();

        drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));

        PoseStorage.currentPose = drive.getPoseEstimate();

        telemetry.addData("Position reset", "finished");
        telemetry.update();

    }
}
