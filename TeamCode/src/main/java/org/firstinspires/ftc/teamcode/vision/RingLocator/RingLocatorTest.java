package org.firstinspires.ftc.teamcode.vision.RingLocator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.vision.RingLocator.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.vision.RingLocator.Dashboard.drawRing;
import static org.firstinspires.ftc.teamcode.vision.RingLocator.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.vision.RingLocator.Dashboard.sendPacket;

@TeleOp(name = "Ring Locator Pipeline Test")
@Disabled
public class RingLocatorTest extends LinearOpMode {

    private RingLocator detector;
    private ArrayList<Ring> rings;
    public static double x = 87;
    public static double y = 63;
    public static double theta = PI/2;

    @Override
    public void runOpMode() {
        detector = new RingLocator(this);
        detector.start();

        waitForStart();

        while (opModeIsActive()) {
            rings = detector.getRings(x, y, theta);

            for (int i = 0; i < rings.size(); i++) {
                if (i == 0) {
                    drawRing(rings.get(i), "green");
                } else if (i == 1) {
                    drawRing(rings.get(i), "yellow");
                } else if (i == 2) {
                    drawRing(rings.get(i), "red");
                }
            }
            drawRobot(x, y, theta, "black");

            addPacket("Rings", rings);
            sendPacket();
        }

        detector.stop();
    }
}