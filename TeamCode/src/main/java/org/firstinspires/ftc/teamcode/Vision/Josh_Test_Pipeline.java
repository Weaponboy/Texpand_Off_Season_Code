package org.firstinspires.ftc.teamcode.Vision;

import android.os.DropBoxManager;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Josh_Test_Pipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public Josh_Test_Pipeline(Telemetry t){telemetry = t;}

    //create matrix's
    Mat workingMatrix = new Mat();
    Mat Purple = new Mat();

    Mat Yellow = new Mat();
    Mat Green = new Mat();

    // region of interest
    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));

    //color threshold
    static double COLOR_THRESHOLD = 0.001;

    //store location
//    public enum Location {
//        purple,
//        green,
//        yellow,
//        not_found
//    }

//    private Location location;

    @Override
    public Mat processFrame(Mat input) {

        //copy to all matrix's
        input.copyTo(workingMatrix);
        input.copyTo(Purple);
        input.copyTo(Yellow);
        input.copyTo(Green);

        //color scales
        Imgproc.cvtColor(Purple, Purple, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Green, Green, Imgproc.COLOR_RGB2HSV);

        Scalar purpleup = new Scalar(320, 100, 255);
        Scalar purpledown = new Scalar(260, 22, 31);
        Scalar greenup = new Scalar(165, 100, 100);
        Scalar greendown = new Scalar(99, 32, 22);
        Scalar yellowup = new Scalar(76, 100, 100);
        Scalar yellowdown = new Scalar(55, 29, 28);

        // check for colors on the matrix's
        Core.inRange(Purple, purpledown, purpleup , Purple);
        Core.inRange(Green, greendown, greenup, Green);
        Core.inRange(Yellow, yellowdown, yellowup, Yellow);

        Purple.submat(center);
        Green.submat(center);
        Yellow.submat(center);

        double PURPLE = Core.sumElems(Purple).val[0] / center.area() / 255;
        double GREEN = Math.round(Core.mean(Green).val[0] / 255);
        double YELLOW = Math.round(Core.mean(Yellow).val[0] / 255);

        Scalar greenrect = new Scalar(0, 204, 0);
        Scalar yellowrect = new Scalar(204, 204, 0);
        Scalar purplerect = new Scalar(76, 0, 153);

        telemetry.addData("purple raw value", (int) Core.mean(Purple).val[0]);
        telemetry.addData("yellow raw value", (int) Core.mean(Yellow).val[0]);
        telemetry.addData("green raw value", (int) Core.mean(Green).val[0]);


        if (YELLOW > COLOR_THRESHOLD){
////            location = Location.yellow;
            Imgproc.rectangle(workingMatrix, center, yellowrect);
            telemetry.addData("Color", "Yellow");
        }else if (GREEN > COLOR_THRESHOLD){
////            location = Location.green;
            Imgproc.rectangle(workingMatrix, center, greenrect);
            telemetry.addData("Color", "Green");
        }else if (PURPLE > COLOR_THRESHOLD){
////            location = Location.purple;
            Imgproc.rectangle(workingMatrix, center, purplerect);
            telemetry.addData("Color", "Purple");
        }
////        else{
////            location = Location.not_found;
////            return input;
////        }
        telemetry.update();


        return workingMatrix;

    }

//    public Location getLocation(){
//        return location;
//    }
}
