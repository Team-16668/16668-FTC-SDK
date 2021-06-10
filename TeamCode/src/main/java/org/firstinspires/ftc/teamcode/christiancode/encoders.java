package org.firstinspires.ftc.teamcode.christiancode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class encoders extends LinearOpMode {
    DcMotor shootMotor;
    @Override

    public void runOpMode() throws InterruptedException {
        shootMotor=hardwareMap.get(DcMotor.class,"shooter");

        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        shootMotor.setPower(.5);
        while(opModeIsActive()){
            double currentposit =shootMotor.getCurrentPosition();
            telemetry.addData("rotations",currentposit/28);
            telemetry.update();
            if(currentposit/28>=300){
                shootMotor.setPower(0);
            }
        }
    }
}
