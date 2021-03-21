package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class TestAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-62, -26, Math.toRadians(0)));

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
        sleep(2000);
        drive.followTrajectory(traj2);
        sleep(2000);
        drive.followTrajectory(traj3);
        sleep(2000);
        drive.followTrajectory(traj4);
        sleep(2000);
        drive.followTrajectory(traj5);
        sleep(2000);
    }
}
