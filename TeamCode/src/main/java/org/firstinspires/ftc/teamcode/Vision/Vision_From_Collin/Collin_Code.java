package org.firstinspires.ftc.teamcode.Vision.Vision_From_Collin;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV_FULL;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.circle;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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
    public Mat output = new Mat(),
            modified = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));
    // Rectangle settings
    private Scalar orange = new Scalar(300, 90, 90);
    private Scalar lightBlue = new Scalar(200, 90, 90);
    public int numcontours;

    private Rect largestRect;
    public int numrects;

    private int font = FONT_HERSHEY_COMPLEX;
    public double rectX;
    public double rectY;

    public double rectCentre;

    public double Distance_To_Travel;

    public double ConversionPixelstoCm = 50;//need to tune this

    public double CenterOfScreen = 640;

    public double rectPositionFromLeft = 0;

    public Scalar MIN_THRESH = new Scalar(135,50,50);
    public Scalar MAX_THRESH = new Scalar(165,255,255);
    public Scalar values;
    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);

        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

//        MIN_THRESH = new Scalar(VisionDash.blue_min_H, VisionDash.blue_min_S, VisionDash.blue_min_V);
//        MAX_THRESH = new Scalar(VisionDash.blue_max_H, VisionDash.blue_max_S, VisionDash.blue_max_V);

        Imgproc.cvtColor(input, modified, COLOR_RGB2HSV_FULL);
        values = Core.mean(modified.submat(center));
        inRange(modified, MIN_THRESH, MAX_THRESH, modified);

        erode(modified, modified, new Mat(VisionDash.erode_const, VisionDash.erode_const, CV_8U));
        dilate(modified, modified, new Mat(VisionDash.dilate_const, VisionDash.dilate_const, CV_8U));

        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }
        numcontours = contours.size();
        if(rects.size() > 0){
            largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);
            rectangle(output, largestRect, lightBlue, 30);
            rectX = largestRect.x + largestRect.width/2;
            rectY = largestRect.y + largestRect.height/2;
            Imgproc.circle(output,new Point(rectX,rectY),50,orange,20);

            rectPositionFromLeft = largestRect.x + largestRect.width/2;
        }

        Distance_To_Travel = rectPositionFromLeft - CenterOfScreen / ConversionPixelstoCm;

        rectX = largestRect.x + largestRect.width/2;
        rectY = largestRect.y + largestRect.height/2;

        numrects = rects.size();
        drawContours(output, contours, -1, orange);
        contours.clear();
        return output;
    }

    public double TravelDistance(){
        return Distance_To_Travel;
    }

    public double rectPositionFromLeft(){
        return rectPositionFromLeft;
    }
}