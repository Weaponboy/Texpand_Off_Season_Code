package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.VisionDash.dilate_const;
import static org.firstinspires.ftc.teamcode.Vision.VisionDash.erode_const;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2YCrCb;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import org.firstinspires.ftc.teamcode.Vision.VisionDash;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Collin_Code extends OpenCvPipeline {

    //Initiated variables needed later
    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;
    private Mat output = new Mat(),
            modified = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    // Rectangle settings
    private Scalar orange = new Scalar(252, 186, 3);
    private Scalar lightBlue = new Scalar(3, 252, 227);

    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);

        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        Scalar MIN_THRESH = new Scalar(VisionDash.blue_min_H, VisionDash.blue_min_S, VisionDash.blue_min_V);
        Scalar MAX_THRESH = new Scalar(VisionDash.blue_max_H, VisionDash.blue_max_S, VisionDash.blue_max_V);

        Imgproc.cvtColor(input, modified, COLOR_RGB2HSV);

        inRange(input, MIN_THRESH, MAX_THRESH, modified);

        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        Rect largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);

        return output;
    }
}
