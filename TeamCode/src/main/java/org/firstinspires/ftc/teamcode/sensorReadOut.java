package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Sensor Read Out")
@Disabled
public class sensorReadOut extends LinearOpMode {

    public BNO055IMU imu;
    public DistanceSensor stone_distance;
    public ColorSensor left_color;
    public ColorSensor right_color;
    public TouchSensor scissor_touch;

    float[] hsv_left = new float[3];
    float[] hsv_right = new float[3];

    @Override
    public void runOpMode() throws InterruptedException {
        stone_distance = hardwareMap.get(DistanceSensor.class, "stone_distance");
        left_color = hardwareMap.get(ColorSensor.class, "left_color");
        right_color = hardwareMap.get(ColorSensor.class, "right_color");

        scissor_touch = hardwareMap.touchSensor.get("scissor_touch");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while(opModeIsActive()) {
            Color.RGBToHSV(left_color.red(), left_color.green(), left_color.blue(), hsv_left);
            Color.RGBToHSV(right_color.red(), right_color.green(), right_color.blue(), hsv_right);
            Orientation turn = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("left color", "h: " + hsv_left[0] + " s: " + hsv_left[1] + " v: " + hsv_left[2]);
            telemetry.addData("left color", "r: " + left_color.red() + " g: " + left_color.green() + " b: " + left_color.blue());
            telemetry.addData("right color", "h: " + hsv_right[0] + " s:" + hsv_right[1] + " v:" + hsv_right[2]);
            telemetry.addData("right color", "r: " + right_color.red() + " g: " + right_color.green() + " b: " + right_color.blue());
            telemetry.addData( "distance", stone_distance.getDistance(DistanceUnit.MM));
            telemetry.addData("touch sensor", scissor_touch.isPressed());
            telemetry.addData("imu", " x: "+turn.firstAngle + " y: " + turn.secondAngle + " z: "+ turn.thirdAngle);
            telemetry.update();

        }
    }
}
