package org.firstinspires.ftc.teamcode.Vision.Piplines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Threshold_Pipeline extends OpenCvPipeline {

    Mat Red = new Mat();
    Mat Yellow = new Mat();
    Mat Blue = new Mat();
    Mat workingmatrix = new Mat();
    static final Rect center = new Rect(new Point(200, 180), new Point(350, 320));

    public boolean Pos_1 = false;
    public boolean Pos_2 = false;
    public boolean Pos_3 = false;

    Telemetry telemetry;

    public Threshold_Pipeline(Telemetry t) {
        telemetry = t;
    }

    public Threshold_Pipeline() {

    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingmatrix);

        input.copyTo(Red);
        input.copyTo(Yellow);
        input.copyTo(Blue);
        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV_FULL);

        Imgproc.cvtColor(Red, Red, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(Blue, Blue, Imgproc.COLOR_RGB2HSV_FULL);

        Scalar redup = new Scalar(250, 100, 100);
        Scalar reddown = new Scalar(150, 0, 0);
        Scalar blueup = new Scalar(148, 100, 100);
        Scalar bluedown = new Scalar(110, 0, 0);
        Scalar yelowup = new Scalar(110, 100, 100);
        Scalar yelowdown = new Scalar(30, 0, 0);

        Core.inRange(Red, reddown, redup, Red);
        Core.inRange(Yellow, yelowdown, yelowup, Yellow);
        Core.inRange(Blue, bluedown, blueup, Blue);


        if (Core.mean(workingmatrix.submat(center)).val[0] > 30 && Core.mean(workingmatrix.submat(center)).val[0] < 110) {
            telemetry.addData("Colour", "Yellow");
            Pos_2 = true;
        } else if (Core.mean(workingmatrix.submat(center)).val[0] > 110 && Core.mean(workingmatrix.submat(center)).val[0] < 135) {
            telemetry.addData("Colour", "Blue");
            Pos_3 = true;
        } else if (Core.mean(workingmatrix.submat(center)).val[0] > 136 && Core.mean(workingmatrix.submat(center)).val[0] < 250) {
            telemetry.addData("Colour", "Red");
            Pos_1 = true;
        }
        telemetry.update();

        Scalar blue = new Scalar(319, 100, 100);

        Imgproc.rectangle(input, center, blue, 10);
        return input.submat(center);
    }

    public Boolean Get_Pos_1(){
        return Pos_2;
    }

    public Boolean Get_Pos_2(){
        return Pos_3;
    }

    public Boolean Get_Pos_3(){

        return Pos_1;
    }
}
