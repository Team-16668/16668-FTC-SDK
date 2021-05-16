package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="turret test")
public class TurretTest extends LinearOpMode {
    WobbleTurret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new WobbleTurret(hardwareMap, WobbleTurret.TurretState.STOWED);

        waitForStart();

        while(opModeIsActive()) {
            turret.wobbleLinearMotor.setPower(gamepad1.right_stick_y);
            telemetry.addData("right", gamepad1.right_stick_y);

            turret.wobbleRotaryMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("left", gamepad1.left_stick_y);
        }
    }

}
