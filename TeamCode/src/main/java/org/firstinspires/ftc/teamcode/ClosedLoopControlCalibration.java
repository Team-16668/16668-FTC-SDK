package org.firstinspires.ftc.teamcode;

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


@TeleOp(name="Closed Loop Control Calibration")
public class ClosedLoopControlCalibration extends LinearOpMode {
    public DcMotorEx shooter;

    double targetRPM = 4350;
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

        waitForStart();
        shooter.setPower(currentPower);

        while(opModeIsActive()) {
            double encoderPosition = shooter.getCurrentPosition();
            totalRevolutions = encoderPosition / 28;
            runTime = (System.nanoTime() - startTime) / TimeUnit.SECONDS.toNanos(1);

            if(runTime - lastTime > 0.05) {

                //Get RPM
                //Last Revolutions Average
                runningTotal = 0;

                revolutionChange = totalRevolutions - lastRevolutions;
                timeChange = runTime - lastTime;
                RPM = (revolutionChange / timeChange) * 60;


                lastRevolutions = totalRevolutions;

                lastTime += 0.05;

                for(int i = LastRPM.length -1; i>0; i--) {
                    LastRPM[i] = LastRPM[i-1];
                }
                LastRPM[0] = RPM;

                runningTotal = 0;

                for(int i = 0; i<LastRPM.length; i++) {
                    runningTotal += LastRPM[i];
                }
                averageRPM = runningTotal/LastRPM.length;



                if(RPM < targetRPM) {
                    currentPower += 0.001;
                    currentPowerMultiplier = 1;
                } else if(RPM > targetRPM) {
                    currentPower -= 0.001;
                    currentPowerMultiplier = -1;
                }
                /*
                if(!contains(LastRPM, 0)) {
                    currentPower += (currentPowerMultiplier * 0.00005*(Math.abs(averageRPM-targetRPM)));
                }
                 */

                if(currentPower > 1) {
                    currentPower = 1;
                } else if(currentPower < 0 ) {
                    currentPower = 0;
                }
            }

            //shooter.setPower(currentPower);
            shooter.setVelocity(2000);

            /*
            for(int i=0; i<LastRPM.length; i++) {
                telemetry.addData("Array index " + i, LastRPM[i]);
            }

             */
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

    void OriginalControl() {
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

            if(averageRPM < targetRPM) {
                currentPower += 0.001;
            } else if(averageRPM > targetRPM) {
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

    public static boolean contains(final double[] array, final double v) {

        boolean result = false;

        for(double i : array){
            if(i == v){
                result = true;
                break;
            }
        }

        return result;
    }

}
