package org.firstinspires.ftc.teamcode.Odometry;

import static java.lang.Math.*;

public class mathFunctions {
    /**
     * Make sure angle is within -180 to 180 degrees
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle) {
       while(angle < -PI) {
           angle += 2 * PI;
       }
       while(angle > PI) {
           angle -= 2 * PI;
       }
       return angle;
    }

    public static boolean isWithin(double value, double min, double max) {
        if(value >= max) {
            return false;
        } else if (value <= min) {
            return false;
        } else {
            return true;
        }
    }

    public static double interpretAngle(double Orientation) {
        if(isWithin(Orientation, -180, 0)) {
            return Orientation;
        } else if (isWithin(Orientation, -360, -180)) {
            double newOrientation = (Orientation +360);
            return newOrientation;
        } else {
            return Orientation;
        }
    }


}
