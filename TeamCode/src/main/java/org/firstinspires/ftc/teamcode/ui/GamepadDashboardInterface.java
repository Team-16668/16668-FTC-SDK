package org.firstinspires.ftc.teamcode.ui;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ftdi.FtConstants;

import java.util.ArrayList;
import java.util.List;

public class GamepadDashboardInterface {
    FtcDashboard dashboard;
    List<Ring> rings;
    Cursor cursor;

    public GamepadDashboardInterface() {
        this.dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        this.rings = new ArrayList<Ring>();
        this.cursor = new Cursor(0, -24, 5);
    }

    public void update(Gamepad gamepad) {
        this.cursor.applyMovement(gamepad.dpad_up, gamepad.dpad_down, gamepad.dpad_left, gamepad.dpad_right, gamepad.left_bumper, gamepad.right_bumper);


    }

    private int onRing(double x, double y) {
        for (Ring temp : rings) {

        }
    }

    private double within(double x1, double y1, double x2, double y2, double allowedDistance) {
        if() {

        }
    }
}
