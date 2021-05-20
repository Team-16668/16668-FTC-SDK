package org.firstinspires.ftc.teamcode.tools.shooterVeloTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.util.InterpLUT;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Closed Loop Tester")
@Disabled
public class ClosedLoopControlCalibration extends LinearOpMode {
    FtcDashboard dashboard;

    public static double distance = 4;

    public DcMotorEx shooter;
    public Servo flicker, backPlate;

    double currentPower = 0.86;

    //Logic for the Flicker
    double flickerStartTime,
            timeSinceFlicker;
    boolean firstReturn = true,
            tryFLick = false;

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

        InterpLUT shooterLut = new InterpLUT();
        shooterLut.add(0,4400);
        shooterLut.add(8,4400);
        shooterLut.add(12,4400);
        shooterLut.add(16,4300);
        shooterLut.add(20,4200);
        shooterLut.add(24,4100);
        shooterLut.add(28,4100);
        shooterLut.add(32,4150);
        shooterLut.add(36,4200);
        shooterLut.add(40,4250);
        shooterLut.add(46,4300);
        shooterLut.add(52,4250);
        shooterLut.add(58,4350);
        shooterLut.createLUT();

        waitForStart();
        shooter.setPower(currentPower);

        while(opModeIsActive()) {

            shooter.setVelocity((shooterLut.get(distance)*28)/60);

            Flick();

            telemetry.addData("Shooter Velocity in RPM", (shooter.getVelocity()/28)*60);
            //telemetry.addData("Shooter Target Velocity in RPM", shooterLut.get(distance));

            telemetry.update();
        }
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
