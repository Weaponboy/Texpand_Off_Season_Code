package org.firstinspires.ftc.teamcode;



import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    
    Mat output = new Mat();
    public enum Location {
        blue,
        red,
        yellow,
        not_found
    }
    private static Location location;

    static final Rect center = new Rect(
            new Point(0, 0),
            new Point(640, 480));

    static double COLOR_THRESHOLD = 0.4;

    @Override
    public Mat processFrame(Mat input) {

        Scalar purpleup = new Scalar(178, 102, 255);
        Scalar purpledown = new Scalar(51, 0, 102);
        Scalar greenup = new Scalar(51, 255, 51);
        Scalar greendown = new Scalar(102, 0, 102);
        Scalar yellowup = new Scalar(255, 255, 102);
        Scalar yellowdown = new Scalar(204, 204, 0);


//        Core.inRange(output, yellow, yellow, output);
//        Core.inRange(output, blue, blue, output);
//        Core.inRange(output, red, red, output);

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
            else if (bluetrue){
                location = Location.blue;
                //telemetry.addData("Color", "blue");
            }
            else if (redtrue){
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




