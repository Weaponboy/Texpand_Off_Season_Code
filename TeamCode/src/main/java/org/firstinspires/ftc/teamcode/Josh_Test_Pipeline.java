package org.firstinspires.ftc.teamcode;

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

    //storing color
    Mat getYellow = new Mat();
    Mat getGreen = new Mat();
    Mat getPurple = new Mat();

    //color threshold
    static double COLOR_THRESHOLD = 0.4;

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

        //to prevent errors
//        if (workingMatrix.empty()){
//            return input;
//        }

        //color scales
//        Imgproc.cvtColor(Purple, Purple, Imgproc.COLOR_HSV2RGB);
//        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_HSV2RGB);
//        Imgproc.cvtColor(Green, Green, Imgproc.COLOR_HSV2RGB);
//        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_HSV2RGB);

        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(Green, Green, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(Purple, Purple, Imgproc.COLOR_RGBA2RGB);
        Scalar purpleup = new Scalar(0, 126, 255);
        Scalar purpledown = new Scalar(77, 0, 255);
        Scalar greenup = new Scalar(51, 255, 51);
        Scalar greendown = new Scalar(102, 0, 102);
        Scalar yellowup = new Scalar(255, 255, 102);
        Scalar yellowdown = new Scalar(204, 204, 0);

        // check for colors on the matrix's
        Core.inRange(Purple, purpleup, purpledown , Purple);
        Core.inRange(Green, greenup, greendown, Green);
        Core.inRange(Yellow, yellowup, yellowdown, Yellow);

        getPurple = Purple.submat(center);
        getGreen = Green.submat(center);
        getYellow = Yellow.submat(center);

        double PURPLE = Math.round(Core.mean(getPurple).val[0] / 255);
        double GREEN = Math.round(Core.mean(getGreen).val[0] / 255);
        double YELLOW = Math.round(Core.mean(getYellow).val[0] / 255);

        getPurple.release();
        getYellow.release();
        getGreen.release();

        Scalar greenrect = new Scalar(0, 204, 0);
        Scalar yellowrect = new Scalar(204, 204, 0);
        Scalar purplerect = new Scalar(76, 0, 153);

        telemetry.addData("purple raw value", (int) Core.mean(getPurple).val[0]);
        telemetry.addData("yellow raw value", (int) Core.mean(getYellow).val[0]);
        telemetry.addData("green raw value", (int) Core.mean(getGreen).val[0]);


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
