/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/

package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class EasyOpenCVVision2var extends OpenCvPipeline {
    // This enum contains the possible number of rings
    public enum ShipPosition {
        LEFT,
        CENTER,
        NONE
    }
    //bpvtytybz
    // Color constants
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    // Constants for determining the existence of shipping element based on the RED content
    final int ONE_SHIP_THRESHOLD = 150;
    final int NONE_SHIP_THRESHOLD = 50;

    // Upper-left point of the rectangle where shipping elemnt will be defined
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(67, 60);
    // The width of the rectangle where the sh. el. will be defined
    static final int REGION_WIDTH = 20;
    // The height of the rectangle where the sh. el. will be defined
    static final int REGION_HEIGHT = 35;

    // Creating the upper-left point of the rectangle where the rings will be defined
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    // Creating the lower-right point of the rectangle where the rings will be defined
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Working variables
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1; // The amount of red in the specified rectangle

    // Working variables
    Mat region2_Cb;
    //Mat Cb2 = new Mat();
    int avg2; // The amount of red in the specified rectangle

    // Working variables
    Mat region3_Cb;
    //Mat Cb2 = new Mat();
    int avg3; // The amount of red in the specified rectangle



    // Upper-left point of the rectangle where shipping elemnt will be defined
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(177, 60);
    // The width of the rectangle where the sh. el. will be defined
    static final int REGION2_WIDTH = 20;
    // The height of the rectangle where the sh. el. will be defined
    static final int REGION2_HEIGHT = 35;

    // Creating the upper-left point of the rectangle where the rings will be defined
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    // Creating the lower-right point of the rectangle where the rings will be defined
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

    // Upper-left point of the rectangle where shipping elemnt will be defined
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(222, 60);
    // The width of the rectangle where the sh. el. will be defined
    static final int REGION3_WIDTH = 20;
    // The height of the rectangle where the sh. el. will be defined
    static final int REGION3_HEIGHT = 35;

    // Creating the upper-left point of the rectangle where the rings will be defined
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    // Creating the lower-right point of the rectangle where the rings will be defined
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);






    // Variable where the number of rings will be stored at the moment
    public volatile ShipPosition position = ShipPosition.NONE;

    // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));

    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        dataFromOpenCV.AVG1=avg1;
        avg2 = (int) Core.mean(region2_Cb).val[0];
        dataFromOpenCV.AVG2=avg2;
        avg3 = (int) Core.mean(region3_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines

        position = ShipPosition.NONE; // Record our analysis

        if (avg1>avg2&&avg1>avg3) {
                position = ShipPosition.LEFT;
        }
        if (avg2>avg1&&avg2>avg3) {
                position = ShipPosition.CENTER;
        }
        if (avg3>avg1&&avg3>avg2) {
            position = ShipPosition.CENTER;
        }














/*
            if (NONE_SHIP_THRESHOLD > avg1) {
                if (NONE_SHIP_THRESHOLD > avg2) {position = ShipPosition.NONE;}
            }
        if (avg1 > avg2) {
            if (avg1 > ONE_SHIP_THRESHOLD) {
                position = ShipPosition.LEFT;
            }
        }
        if (avg2 > avg1) {
            if (avg2 > ONE_SHIP_THRESHOLD) {
                position = ShipPosition.CENTER;
            }
        }*/
        // else if (NONE_SHIP_THRESHOLD > avg1 > avg2) {
        //   position = ShipPosition.NONE; }

//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region1_pointA, // First point which defines the rectangle
//                region1_pointB, // Second point which defines the rectangle
//                GREEN, // The color the rectangle is drawn in
//                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis() {
        return avg1;
    }
    public int getAnalysis2() {
        return avg2;
    }

}


