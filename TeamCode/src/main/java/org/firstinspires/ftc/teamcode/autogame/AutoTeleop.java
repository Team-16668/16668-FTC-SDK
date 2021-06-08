package org.firstinspires.ftc.teamcode.autogame;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autogame.ui.GamepadDashboardInterface;


@TeleOp(name="Autonomous Teleop")
public class AutoTeleop extends LinearOpMode {
    //TODO: Make the entire robot subsystem so there aren't tons of functions shoved here

    //MecanumDriveAuto drive;
    GamepadDashboardInterface ui;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        //drive = new MecanumDriveAuto(hardwareMap, ui);
        ui = new GamepadDashboardInterface();
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(opModeIsActive()) {
            ui.update(gamepad1);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("cursor x", ui.cursor.getX());
            packet.put("cursor y", ui.cursor.getY());
            packet.put("rings", ui.rings.size());
            packet.put("up", ui.gamepad.dpad_up);
            packet.put("down", ui.gamepad.dpad_down);
            packet.put("left", ui.gamepad.dpad_left);
            packet.put("right", ui.gamepad.dpad_right);
            packet.put("adjusted speed", ui.cursor.adjustedSpeed);

            ui.drawUI(fieldOverlay);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
