package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class PowerShotTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvWebcam webcam;
    CustomPowershotPipelineRed pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "goal_webcam"), cameraMonitorViewId);
        pipeline = new CustomPowershotPipelineRed(60, 0, 0);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        dashboard.startCameraStream(webcam, 0);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("left Visisble", pipeline.leftPowerShotVisible());
            telemetry.addData("left Angle", pipeline.calculateYaw(CustomPowershotPipelineRed.Target.LEFT));
            telemetry.addData("Center Visisble", pipeline.centerPowerShotVisible());
            telemetry.addData("Center angle", pipeline.calculateYaw(CustomPowershotPipelineRed.Target.CENTER));
            telemetry.addData("Right Visisble", pipeline.rightPowerShotVisible());
            telemetry.addData("Right Angle", pipeline.calculateYaw(CustomPowershotPipelineRed.Target.RIGHT));
            telemetry.update();
        }
    }
}
