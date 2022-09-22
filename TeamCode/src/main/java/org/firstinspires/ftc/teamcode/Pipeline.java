package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    public Pipeline() { }
    
    Mat output = new Mat();
    public enum Location {
        blue,
        red,
        yellow,
        not_found
    }
    private Location location;
    static final Rect center = new Rect(
            new Point(35, 75),
            new Point(75, 35));

    static double COLOR_THRESHOLD = 0.4;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);
        Scalar yellow = new Scalar(59, 93 ,100);
        Scalar blue = new Scalar(235, 90, 100);
        Scalar red = new Scalar(0, 98, 100);

        Core.inRange(output, yellow, yellow, output);
        Core.inRange(output, blue, blue, output);
        Core.inRange(output, red, red, output);

        Mat yellowcheck = output.submat(center);
        Mat bluecheck = output.submat(center);
        Mat redcheck = output.submat(center);

        double YELLOW = Core.sumElems(yellowcheck).val[0] / center.area() / 255;
        double BLUE = Core.sumElems(yellowcheck).val[0] / center.area() / 255;
        double RED = Core.sumElems(yellowcheck).val[0] / center.area() / 255;

        yellowcheck.release();
        bluecheck.release();
        redcheck.release();

        boolean yellowtrue = YELLOW > COLOR_THRESHOLD;
        boolean bluetrue = BLUE > COLOR_THRESHOLD;
        boolean redtrue = RED > COLOR_THRESHOLD;

            if (yellowtrue){
                location = Location.yellow;
                //telemetry.addData("Color", "yellow");
            }
            if (bluetrue){
                location = Location.blue;
                //telemetry.addData("Color", "blue");
            }
            if (redtrue){
                location = Location.red;
                //telemetry.addData("Color", "red" );
            }else{
                location = Location.not_found;
                //telemetry.addData("Color", "Not found" );
            }
        //telemetry.update();

        Imgproc.cvtColor(input, output, Imgproc.COLOR_GRAY2RGB);

        Scalar bluerect = new Scalar(0, 0, 250);
        Scalar yellowrect = new Scalar(0, 250, 0);
        Scalar redrect = new Scalar(250, 0, 0);

        Imgproc.rectangle(output, center, location == Location.blue? bluerect:yellowrect);
        Imgproc.rectangle(output, center, location == Location.yellow? yellowrect:bluerect);
        Imgproc.rectangle(output, center, location == Location.red? redrect:bluerect);

        return output;

    }

//    public Location getLocation(){
//        return location;
//    }
}




