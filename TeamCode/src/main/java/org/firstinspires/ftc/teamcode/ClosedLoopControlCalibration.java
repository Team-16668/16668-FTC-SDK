package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Closed Loop Tester")
public class ClosedLoopControlCalibration extends LinearOpMode {
    FtcDashboard dashboard;

    public static double shooterSpeed = 4350;
    public static double distance = 4;

    public double velocity;

    public DcMotorEx shooter;
    public Servo flicker, backPlate;

    double totalRevolutions;
    double runTime = 0;
    double currentPower = 0.86;

    //Logic for the Flicker
    double flickerStartTime,
            timeSinceFlicker;
    boolean firstReturn = true,
            tryFLick = false;

    File powerFile = AppUtil.getInstance().getSettingsFile("custom-power.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flicker = hardwareMap.servo.get("flicker");
        backPlate = hardwareMap.servo.get("backplate");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backPlate.setPosition(0);
        flicker.setPosition(1);

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        shooter.setPower(currentPower);

        while(opModeIsActive()) {
            velocity = 0.000002*Math.pow(distance, 6) - 0.0004*Math.pow(distance, 5) + 0.0226*Math.pow(distance, 4) - 0.5438*Math.pow(distance, 3) + 4.5513*Math.pow(distance, 2) - 9.4781*distance + 4398.4;
            shooter.setVelocity((velocity*28)/60);

            Flick();

            telemetry.addData("Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Velocity in RPM", (shooter.getVelocity()/28)*60);
            telemetry.addData("Shooter Target Velocity in RPM", shooterSpeed);

            telemetry.addData("Power", currentPower);
            telemetry.addData("Revolutions", totalRevolutions);
            telemetry.addData("Runtime (Sec)", runTime);

            telemetry.update();
        }
        ReadWriteFile.writeFile(powerFile, String.valueOf(currentPower));
    }

    void Flick() {
        timeSinceFlicker = (System.nanoTime() - flickerStartTime) / TimeUnit.SECONDS.toNanos(1);
        if(timeSinceFlicker >= 0.5) {
            if(firstReturn) {
                flicker.setPosition(1);
                firstReturn = false;
            }
        }
        tryFLick = gamepad1.dpad_right || gamepad1.dpad_left;
        if (timeSinceFlicker >= 1 && tryFLick) {
            flicker.setPosition(0);
            flickerStartTime = System.nanoTime();
            firstReturn = true;
        }
    }
}
