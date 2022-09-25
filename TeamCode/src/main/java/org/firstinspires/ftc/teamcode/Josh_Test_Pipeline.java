package org.firstinspires.ftc.teamcode;

import android.os.DropBoxManager;

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
    static final Rect center = new Rect(new Point(0, 0), new Point(640, 480));

    //storing color
    Mat getYellow;
    Mat getGreen;
    Mat getPurple;

    //color threshold
    static double COLOR_THRESHOLD = 0.4;

    //store location
    public enum Location {
        purple,
        green,
        yellow,
        not_found
    }

    private Location location;

    @Override
    public Mat processFrame(Mat input) {
        //copy to all matrix's
        input.copyTo(workingMatrix);
        input.copyTo(Purple);
        input.copyTo(Yellow);
        input.copyTo(Green);

        if (workingMatrix.empty()){
            return input;
        }

        Scalar purpleup = new Scalar(178, 102, 255);
        Scalar purpledown = new Scalar(51, 0, 102);
        Scalar greenup = new Scalar(51, 255, 51);
        Scalar greendown = new Scalar(102, 0, 102);
        Scalar yellowup = new Scalar(255, 255, 102);
        Scalar yellowdown = new Scalar(204, 204, 0);

        Core.inRange(Purple, purpleup, purpledown, Purple);
        Core.inRange(Green, greenup, greendown, Green);
        Core.inRange(Yellow, yellowup, yellowdown, Yellow);

        getPurple = Purple.submat(center);
        getGreen = Green.submat(center);
        getYellow = Yellow.submat(center);


        double YELLOW = Core.mean(getPurple).val[2] / 255;
        double GREEN = Core.sumElems(getGreen).val[2] / 255;
        double PURPLE = Core.sumElems(getYellow).val[2] / 255;

        getPurple.release();
        getYellow.release();
        getGreen.release();
        workingMatrix.release();

        boolean yellowtrue = YELLOW > COLOR_THRESHOLD;
        boolean greentrue = GREEN > COLOR_THRESHOLD;
        boolean purpletrue = PURPLE > COLOR_THRESHOLD;

        Scalar greenrect = new Scalar(0, 204, 0);
        Scalar yellowrect = new Scalar(204, 204, 0);
        Scalar purplerect = new Scalar(76, 0, 153);

        if (yellowtrue){
            location = Location.yellow;
            Imgproc.rectangle(workingMatrix, center, yellowrect);
            telemetry.addData("Color", "Yellow");
        }else if (greentrue){
            location = Location.green;
            Imgproc.rectangle(workingMatrix, center, greenrect);
            telemetry.addData("Color", "Green");
        }else if (purpletrue){
            location = Location.purple;
            Imgproc.rectangle(workingMatrix, center, purplerect);
            telemetry.addData("Color", "Purple");
        }else{
            location = Location.not_found;
            return input;
        }
        telemetry.update();

        return workingMatrix;

    }

    public Location getLocation(){
        return location;
    }
}
