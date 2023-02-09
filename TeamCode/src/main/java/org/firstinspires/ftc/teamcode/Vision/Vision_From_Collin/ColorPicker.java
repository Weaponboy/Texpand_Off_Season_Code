package org.firstinspires.ftc.teamcode.Vision.Vision_From_Collin;

import static org.opencv.imgproc.Imgproc.COLOR_RGB2YCrCb;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorPicker extends OpenCvPipeline {

    //Initiated variables needed later
    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;
    private Mat output = new Mat(),
            modified = new Mat(),
            red = new Mat(),
            yellow = new Mat();
    private ArrayList<MatOfPoint> redContours, yellowContours;
    private Mat hierarchy = new Mat();

    // Rectangle settings
    private Scalar orange = new Scalar(252, 186, 3);
    private Scalar lightBlue = new Scalar(3, 252, 227);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(output);


        IMG_HEIGHT = input.rows() / 2;
        IMG_WIDTH = input.cols() / 2;
        Point center = new Point(IMG_WIDTH, IMG_HEIGHT);
        circle(output, center, 5, orange, thickness);
        Imgproc.cvtColor(input, modified, COLOR_RGB2YCrCb);

        VisionDash.YCbCrReadout = modified.get(IMG_HEIGHT, IMG_WIDTH);
        return output;
    }
}