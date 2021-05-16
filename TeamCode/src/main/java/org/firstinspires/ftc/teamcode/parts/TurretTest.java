package org.firstinspires.ftc.teamcode.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="turret test")
public class TurretTest extends LinearOpMode {
    public static double linearMultiplier = 0.25, rotaryMultiplier = 0.25;

    WobbleTurret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new WobbleTurret(hardwareMap, WobbleTurret.TurretState.STOWED);

        waitForStart();

        while(opModeIsActive()) {
            turret.wobbleLinearMotor.setPower(gamepad1.right_stick_y*linearMultiplier);
            telemetry.addData("right", gamepad1.right_stick_y*linearMultiplier);

            turret.wobbleRotaryMotor.setPower(gamepad1.left_stick_y*rotaryMultiplier);
            telemetry.addData("left", gamepad1.left_stick_y*rotaryMultiplier);

            turret.clawLift.setPower(gamepad1.right_trigger);

            telemetry.update();
        }
    }

}
