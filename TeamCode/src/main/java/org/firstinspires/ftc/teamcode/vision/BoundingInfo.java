package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Rect;

public class BoundingInfo {
    public Rect boundingBox;
    public double boundingCenterX;

    public BoundingInfo(Rect boundingBox, double boundingCenterX) {
        this.boundingBox = boundingBox;
        this.boundingCenterX = boundingCenterX;
    }


}
