package org.firstinspires.ftc.teamcode.Vision;

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

public class CustomPowershotPipelineRed extends OpenCvPipeline {

    protected double centerX;
    protected double centerY;

    public int minThreshold, maxThreshold;
    private Mat blueThreshold;
    private Mat redThreshold;

    private Mat matYCrCb;
    private Mat redChannel;
    private Mat blueChannel;

    private List<MatOfPoint> redContours;
    private List<MatOfPoint> blueContours;
    private MatOfPoint biggestBlueContour;
    private MatOfPoint biggestRedContour;
    private Rect blueRect, redRect;
    private Rect powerShotLeft, powerShotCenter, powerShotRight;
    private Rect[] boundingBoxes;
    private BoundingInfo[] boundingInfo;

    public CustomPowershotPipelineRed() {
        matYCrCb = new Mat();
        redChannel = new Mat();
        blueChannel = new Mat();

        blueThreshold = new Mat();
        redThreshold = new Mat();

        blueContours = new ArrayList<MatOfPoint>();
        redContours = new ArrayList<MatOfPoint>();

        biggestBlueContour = new MatOfPoint();
        biggestRedContour = new MatOfPoint();

        blueRect = new Rect();
        redRect = new Rect();

        boundingBoxes = new Rect[0];
        boundingInfo = new BoundingInfo[0];

        minThreshold = 155;
        maxThreshold = 200;
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        int imageWidth = mat.width();
        int imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;
    }

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 30;
    }

    @Override
    public Mat processFrame(Mat input) {
        //TODO: Add Blue Code because for now I'm just doing red
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(matYCrCb, redChannel, 1);

        // Red threshold
        Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);

        redContours.clear();

        Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        redContours = redContours.stream().filter(i -> {
            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width > 4)
                    && ((double) Imgproc.boundingRect(i).height / Imgproc.boundingRect(i).width < 6);
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

        for(int i=0; i<redContours.size(); i++) {
            boundingInfo[i].boundingBox = Imgproc.boundingRect(redContours.get(i));
            boundingInfo[i].boundingCenterX = boundingInfo[i].boundingBox.x + boundingInfo[i].boundingBox.width;
        }

        boundingInfo = Sort(boundingInfo);

        if(boundingInfo.length > 0) {
            powerShotLeft = boundingInfo[0].boundingBox;
        } else {
            powerShotLeft = null;
        }
        if(boundingInfo.length > 1) {
            powerShotCenter = boundingInfo[1].boundingBox;
        } else {
            powerShotCenter = null;
        }
        if(boundingInfo.length > 2) {
            powerShotRight = boundingInfo[2].boundingBox;
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

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public BoundingInfo[] Sort(BoundingInfo boundingInfo[]) {
        boolean sorted = false;
        while(!sorted) {
            sorted = false;
            for(int i = 0; i< boundingInfo.length; i++) {
                if(boundingInfo[i].boundingCenterX < boundingInfo[i-1].boundingCenterX) {
                    BoundingInfo tempBoundingInfo = boundingInfo[i-1];
                    boundingInfo[i-1] = boundingInfo[i];
                    boundingInfo[i] = tempBoundingInfo;
                    sorted = true;
                }
            }
        }
        return boundingInfo;
    }


}
