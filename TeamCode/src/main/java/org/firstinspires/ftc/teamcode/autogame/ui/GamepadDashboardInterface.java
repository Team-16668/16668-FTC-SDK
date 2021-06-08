package org.firstinspires.ftc.teamcode.autogame.ui;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.*;

public class GamepadDashboardInterface {
    FtcDashboard dashboard;
    public List<Ring> rings;
    public Cursor cursor;
    int selectedRing;

    public Gamepad gamepad;

    boolean prevA;
    boolean prevB;

    public ElapsedTime timer;

    String orange = "FFA500";
    String blue = "#0000FF";

    public GamepadDashboardInterface() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        rings = new ArrayList<>();
        cursor = new Cursor(0, -24, 1);
        selectedRing = -1;

        timer = new ElapsedTime();
        timer.reset();
    }

    public void update(Gamepad gamepad) {
        this.gamepad = gamepad;
        if(timer.seconds() > 0.05) {
            cursor.applyMovement(gamepad.dpad_up, gamepad.dpad_down, gamepad.dpad_left, gamepad.dpad_right, gamepad.left_bumper, gamepad.right_bumper);
            timer.reset();
        }

        shouldCreateRing(gamepad.a);
        shouldDeleteRing(gamepad.b);

        resetRingColors();
    }

    private void resetRingColors() {
        for(int i=0; i<rings.size(); i++) {
            if(rings.get(i).getColor() == blue && i != selectedRing) {
                rings.get(i).setColor(orange);
            }
        }
    }

    private void shouldCreateRing(boolean a) {
        if(a && prevA != a) { createRing(); }
        prevA = a;
    }

    private void shouldDeleteRing(boolean b) {
        selectedRing = onRing();
        if(selectedRing != -1) {
            if(b && prevB != b) { rings.remove(selectedRing); }
        }
        prevB = b;
    }

    private void createRing() {
        rings.add(new Ring(cursor.getX(), cursor.getY(), orange, false));
    }

    private int onRing() {
        int activeIndex = -1;
        for (int i = 0; i<rings.size(); i++) {
            Ring temp = rings.get(i);
            if(withinRange(temp.getX(), temp.getY(), cursor.getX(), cursor.getY(), 5)) {
                rings.get(i).setColor(blue);
                activeIndex = i;
                break;
            }
        }
        return activeIndex;
    }

    private boolean withinRange(double x1, double y1, double x2, double y2, double allowedDistance) {
        if(sqrt(pow(x2-x1, 2) + pow(y2-y1, 2)) < allowedDistance) {
            return true;
        } else {
            return false;
        }
    }

    public void drawUI(Canvas canvas) {
        canvas.setStrokeWidth(2);
        for(int i=0; i<rings.size(); i++) {
            canvas.setStroke(rings.get(i).getColor());
            canvas.strokeCircle(rings.get(i).getX(), rings.get(i).getY(), 5);
        }

        double lineLength = 10;
        double x = cursor.getX();
        double y = cursor.getY();

        canvas.setStroke("#FF0000");
        canvas.strokeLine(x + lineLength, y, x - lineLength, y);
        canvas.strokeLine(x, y + lineLength, x, y - lineLength);
    }
}
