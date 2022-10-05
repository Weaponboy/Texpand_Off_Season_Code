package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class blue_Pipeline extends OpenCVPipeline{

    Telemetry telemetry;
    public blue_Pipeline(Telemetry t){telemetry = t;}

    //create matrix's
    Mat workingMatrix = new Mat();
    Mat Blue = new Mat();

    // region of interest
    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));

    //storing color
    Mat getBlue = new Mat();

    //color threshold
    static double COLOR_THRESHOLD = 0.4;

    @Override
    public Mat processFrame(Mat input) {

        //copy to all matrix's
        input.copyTo(workingMatrix);
        input.copyTo(Blue);

        Imgproc.cvtColor(Blue, Blue, Imgproc.COLOR_RGBA2RGB);

        Scalar purpleup = new Scalar(0, 0, 255);
        Scalar purpledown = new Scalar(0, 0, 0);

        Core.inRange(Blue, purpleup, purpledown , Blue);

        getBlue = Blue.submat(center);

        double BLUE = Math.round(Core.mean(getBlue).val[2] / 255);

        getBlue.release();

        Scalar yellowrect = new Scalar(0, 250, 0);

        telemetry.addData("purple raw value", (int) Core.mean(getBlue).val[2]/255);

        if (BLUE > COLOR_THRESHOLD) {
            Imgproc.rectangle(workingMatrix, center, yellowrect);
            telemetry.addData("Color", "Yellow");
        }
        telemetry.update();

        return workingMatrix;
    }

}
