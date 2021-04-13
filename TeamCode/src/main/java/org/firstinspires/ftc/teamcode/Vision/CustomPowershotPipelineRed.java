package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class CustomPowershotPipelineRed extends OpenCvPipeline {

    public static double minRatio;
    public static double maxRatio;
    public static double filterArea;
    public static int usableHeight;
    public static int usableWidth;
    public static int startX;
    public static int startY;

    protected double centerX;
    protected double centerY;

    private int imageWidth;
    private int imageHeight;

    private double horizontalFocalLength;

    public static int minThreshold, maxThreshold;
    private Mat redThreshold;

    private Mat matYCrCb;
    private Mat redChannel;
    private Rect redRect;

    private List<MatOfPoint> redContours;
    private Rect powerShotLeft, powerShotCenter, powerShotRight;
    private Rect[] boundingBoxes;
    private BoundingInfo[] boundingInfo;

    double fov;
    double cameraPitchOffset;
    double cameraYawOffset;

    public CustomPowershotPipelineRed(double fov, double cameraPitchOffset, double cameraYawOffset) {
        matYCrCb = new Mat();
        redChannel = new Mat();

        redThreshold = new Mat();

        redContours = new ArrayList<MatOfPoint>();

        redRect = new Rect();

        boundingBoxes = new Rect[0];
        boundingInfo = new BoundingInfo[0];

        minThreshold = 155;
        maxThreshold = 200;

        minRatio = 2;
        maxRatio = 7;
        filterArea = 25;
        usableHeight = 240;
        usableWidth = 320;
        startX = 0;
        startY = 0;

        this.fov = fov;
        this.cameraPitchOffset = cameraPitchOffset;
        this.cameraYawOffset = cameraYawOffset;
    }

    public CustomPowershotPipelineRed() {
        this(60, 0, 0);
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        imageWidth = mat.width();
        imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;

        double diagonalView = Math.toRadians(this.fov);

        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;

        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
    }

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > filterArea;
    }

    @Override
    public Mat processFrame(Mat input) {
        //TODO: Add Blue Code because for now I'm just doing red
        input = new Mat(input, new Rect(startX, startY, usableWidth, usableHeight));

        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(matYCrCb, redChannel, 1);

        // Red threshold
        Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
        saveMatToDisk(redThreshold, "threshold");

        redContours.clear();

        Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        redContours = redContours.stream().filter(i -> {
            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width > minRatio)
                    && ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width < maxRatio);
            return filterContours(i) && appropriateAspect;
        }).collect(Collectors.toList());

        Imgproc.drawContours(input, redContours, -1, new Scalar(255, 255, 0));

        /*
        if (!redContours.isEmpty()) {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            redRect = Imgproc.boundingRect(biggestRedContour);
            Imgproc.rectangle(input, redRect, new Scalar(255, 0, 0), 3);
        } else {
            redRect = null;
        }
         */
        boundingInfo = new BoundingInfo[redContours.size()];

        if(redContours.size() > 0 && redContours.get(0) != null) {
            for(int i=0; i<redContours.size(); i++) {
                try {
                    Rect rect = Imgproc.boundingRect(redContours.get(i));
                    double centerX = rect.x + rect.width;

                    boundingInfo[i] = new BoundingInfo(rect, centerX);
                } catch (Exception e) {
                    e.printStackTrace();
                }

            }
        }


        boundingInfo = Sort(boundingInfo);

        if(boundingInfo.length > 0) {
            powerShotLeft = boundingInfo[0].boundingBox;
            Imgproc.rectangle(input, powerShotLeft, new Scalar(255, 0, 0), 2);
        } else {
            powerShotLeft = null;
        }
        if(boundingInfo.length > 1) {
            powerShotCenter = boundingInfo[1].boundingBox;
            Imgproc.rectangle(input, powerShotCenter, new Scalar(0, 255, 0), 2);
        } else {
            powerShotCenter = null;
        }
        if(boundingInfo.length > 2) {
            powerShotRight = boundingInfo[2].boundingBox;
            Imgproc.rectangle(input, powerShotRight, new Scalar(0, 0, 255), 2);
        } else {
            powerShotRight = null;
        }


        return input;
    }

    public Rect getPowerShotLeft() {
        return powerShotLeft;
    }
    public Rect getPowerShotCenter() {
        return powerShotCenter;
    }
    public Rect getPowerShotRight() {
        return powerShotRight;
    }

    public boolean arePowershotsVisible() {
        return (powerShotLeft != null && powerShotCenter != null && powerShotRight != null);
    }

    public boolean leftPowerShotVisible() { return (powerShotLeft != null); }
    public boolean centerPowerShotVisible() { return (powerShotCenter != null); }
    public boolean rightPowerShotVisible() { return (powerShotRight != null); }

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public double calculateYaw(Target powerShot) {
        return calculateYaw(powerShot, centerX) + cameraYawOffset;
    }

    public double calculateYaw(Target powerShot, double offsetCenterX) {
        Rect currentRect;
        if(powerShot == Target.LEFT) {
            currentRect = powerShotLeft;
        } else if(powerShot == Target.CENTER) {
            currentRect = powerShotCenter;
        } else {
            currentRect = powerShotRight;
        }
        double targetCenterX = getCenterofRect(currentRect).x;

        return Math.toDegrees(
                Math.atan((targetCenterX - offsetCenterX) / horizontalFocalLength)
        );
    }

    public BoundingInfo[] Sort(BoundingInfo boundingInfo[]) {
        boolean sorted = false;
        while(!sorted) {
            sorted = true;
            for(int i = 1; i< boundingInfo.length; i++) {
                if(boundingInfo[i].boundingCenterX < boundingInfo[i-1].boundingCenterX) {
                    BoundingInfo tempBoundingInfo = boundingInfo[i-1];
                    boundingInfo[i-1] = boundingInfo[i];
                    boundingInfo[i] = tempBoundingInfo;
                    sorted = false;
                }
            }
        }
        return boundingInfo;
    }

    public enum Target { LEFT, CENTER, RIGHT}

    class Fraction {
        private int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

}
