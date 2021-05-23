package org.firstinspires.ftc.teamcode.vision;

import android.nfc.NfcAdapter;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;


@Config
public class StackDetection extends OpenCvPipeline {

    //The different possibilities for the starter stack
    public enum StackHeight {
        ZERO,
        ONE,
        FOUR
    }

    final double oneRingRatio = 1;
    final double fourRingRatio = 4;

    //Volatile because this is accessed in the code w/o syncrhonization.
    private volatile StackHeight height;

    Mat region1_Cb;
    Mat YCrCb;
    Mat Cb;
    Mat threshold;
    public static double maxTrheshold;
    public static double minThreshold;
    private double ratio = 0;
    List<MatOfPoint> ringContours;
    MatOfPoint biggestRingContour;
    Rect ringRect;
    int avg1;


    public StackDetection() {
        height = StackHeight.FOUR;

         YCrCb = new Mat();
         Cb = new Mat();
         threshold = new Mat();

         ringContours = new ArrayList<MatOfPoint>();

         biggestRingContour = new MatOfPoint();

         ringRect = new Rect();

         minThreshold = 100;
         maxTrheshold = 125;
         ratio = 0;
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToCB(input);

        Imgproc.threshold(Cb, threshold, minThreshold, maxTrheshold, Imgproc.THRESH_BINARY);

        ringContours.clear();

        Imgproc.findContours(threshold, ringContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        ringContours = ringContours.stream().filter(i -> {
            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height > 0.5)
                    && ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height < 5);
            return filterContours(i) && appropriateAspect;
        }).collect(Collectors.toList());

        Imgproc.drawContours(input, ringContours, -1, new Scalar(255, 165, 0));

        if (!ringContours.isEmpty()) {
            biggestRingContour = Collections.max(ringContours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            ringRect = Imgproc.boundingRect(biggestRingContour);
            Imgproc.rectangle(input, ringRect, new Scalar(255, 0, 0), 3);
        } else {
            ringRect = null;
        }

        if(ringRect != null) {
            ratio = (double) ringRect.width / ringRect.height;
            if(ratio > 2 && ratio < 4) {
                height = StackHeight.ONE;
            } else if(ratio < 2 && ratio > 1) {
                height = StackHeight.FOUR;
            } else {
                height = StackHeight.ZERO;
            }
        } else {
            height = StackHeight.ZERO;
        }


        //I'm returning this thresholded Mat for now because I'm not sure what values I'll need to use for it.
        return threshold;
    }

    public boolean filterContours(MatOfPoint contour) {
        return Imgproc.contourArea(contour) > 30;
    }

    /**
     * This takes the existing image and extrats the red channel from it.
     * @param image - The original image in RGB colorspace
     */
    void inputToCB(Mat image) {
        Imgproc.cvtColor(image, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }


    public StackHeight getHeight() {
        return height;
    }

    public double getRatio() {
        return ratio;
    }

}
