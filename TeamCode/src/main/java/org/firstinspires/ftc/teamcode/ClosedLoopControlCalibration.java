package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.TimeUnit;


@TeleOp(name="Closed Loop Control Calibration")
public class ClosedLoopControlCalibration extends LinearOpMode {
    public DcMotor shooter;


    File powerFile = AppUtil.getInstance().getSettingsFile("custom-power.txt");

    public void runOpMode() throws InterruptedException {
        double targetRPM = 4300;
        double totalRevolutions;
        double runTime = 0;
        double currentPower = 0.9;

        shooter = hardwareMap.dcMotor.get("shooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        shooter.setPower(currentPower);

        double startTime = System.nanoTime();
        double lastTime = 0;
        double lastRevolutions = 0;
        double RPM = 0;
        double revolutionChange, timeChange;

        while(opModeIsActive()) {
            double encoderPosition = shooter.getCurrentPosition();
            totalRevolutions = encoderPosition / 28;
            runTime = (System.nanoTime() - startTime) / TimeUnit.SECONDS.toNanos(1);

            if(runTime - lastTime > 0.05) {

                //Get RPM

                revolutionChange = totalRevolutions - lastRevolutions;
                timeChange = runTime - lastTime;
                RPM = (revolutionChange / timeChange) * 60;

                lastRevolutions = totalRevolutions;
                lastTime += 0.05;

                if(RPM < targetRPM) {
                    currentPower += 0.001;
                } else if(RPM > targetRPM) {
                    currentPower -= 0.001;
                }

                if(currentPower > 1) {
                    currentPower = 1;
                } else if(currentPower < 0 ) {
                    currentPower = 0;
                }
            }

            shooter.setPower(currentPower);

            telemetry.addData("Power", currentPower);
            telemetry.addData("Revolutions", totalRevolutions);
            telemetry.addData("Runtime (Sec)", runTime);
            telemetry.addData("RPM", RPM);
            telemetry.update();
        }
        ReadWriteFile.writeFile(powerFile, String.valueOf(currentPower));
    }

}
