package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.TimeUnit;

import java.lang.Math;

@Config
@TeleOp(name="Closed Loop Control Calibration")
public class ClosedLoopControlCalibration extends LinearOpMode {
    public DcMotorEx shooter;

    public static double targetRPM = 4350;
    double totalRevolutions;
    double runTime = 0;
    double currentPower = 0.86;

    double startTime = System.nanoTime();
    double lastTime = 0;
    double lastRevolutions = 0;
    double RPM = 0;
    double revolutionChange, timeChange;

    double[] LastRPM = {0,0,0,0,0,0,0,0,0,0};
    double runningTotal = 0;
    double averageRPM;
    double currentPowerMultiplier = 1;

    File powerFile = AppUtil.getInstance().getSettingsFile("custom-power.txt");

    public void runOpMode() throws InterruptedException {


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        shooter.setPower(currentPower);

        while(opModeIsActive()) {
            shooter.setVelocity((targetRPM*28)/60);


            telemetry.addData("Last Average", lastRevolutions);
            telemetry.addData("Revolution Change", revolutionChange);

            telemetry.addData("Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Velocity in RPM", (shooter.getVelocity()/28)*60);

            telemetry.addData("Power", currentPower);
            telemetry.addData("Revolutions", totalRevolutions);
            telemetry.addData("Runtime (Sec)", runTime);
            telemetry.addData("RPM", RPM);

            telemetry.update();
        }
        ReadWriteFile.writeFile(powerFile, String.valueOf(currentPower));
    }
}
