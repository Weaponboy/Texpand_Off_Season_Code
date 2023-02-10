package org.firstinspires.ftc.teamcode.Vision.Cone_Alignment;

import static org.firstinspires.ftc.teamcode.Vision.Vision_From_Collin.VisionDash.dilate_const;
import static org.firstinspires.ftc.teamcode.Vision.Vision_From_Collin.VisionDash.erode_const;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV_FULL;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.firstinspires.ftc.teamcode.Vision.Vision_From_Collin.VisionUtils;
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

public class Pole_Pipe extends OpenCvPipeline {

    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;

    //Create Mats
    public Mat output = new Mat(),
            modified = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));

    // Rectangle colour Scales
    private Scalar orange = new Scalar(300, 90, 90);
    private Scalar lightBlue = new Scalar(200, 90, 90);

    public int numcontours;
    public int numrects;


    private int font = FONT_HERSHEY_COMPLEX;
    private double rectX;

    private double H;

    private double S;

    private double V;

    //create rects
    static final Rect center2 = new Rect(new Point(200, 180), new Point(350, 320));
    private double rectY;

    Rect rect = new Rect(new Point(0,0), new Point(50, 50));
    private Rect largestRect;
    private List<Rect> rects = new ArrayList<>();

    //colour scales for the cone
    public Scalar MIN_THRESH = new Scalar(40,40,80);
    public Scalar MAX_THRESH = new Scalar(85,200,200);

    public Scalar values;

    @Override
    public Mat processFrame(Mat input) {

        // copy input to output
        input.copyTo(output);

        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

//        MIN_THRESH = new Scalar(VisionDash.blue_min_H, VisionDash.blue_min_S, VisionDash.blue_min_V);
//        MAX_THRESH = new Scalar(VisionDash.blue_max_H, VisionDash.blue_max_S, VisionDash.blue_max_V);

        //convert to HSV FULL
        Imgproc.cvtColor(input, modified, COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(output, output, COLOR_RGB2HSV_FULL);

        //submat
        values = Core.mean(modified.submat(center));

        //Apply colour scales
        inRange(modified, MIN_THRESH, MAX_THRESH, modified);

        //erode image
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));

        //dilate image
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        //find outlines of the objects of the colours in the range
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        //add contours and create rects
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        numcontours = contours.size();

        //find largest rect and draw a rectangle and mark it with a circle in the center
        if(rects.size() > 0){

            largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);

            rectangle(output, largestRect, lightBlue, 30);

            rectX = largestRect.x + largestRect.width/2;
            rectY = largestRect.y + largestRect.height/2;

            Imgproc.circle(output,new Point(rectX,rectY),50,orange,20);
        }

        //This is used to output the HSV For debugging
        H = Core.mean(output.submat(center2)).val[0];

        S = Core.mean(output.submat(center2)).val[1];

        V = Core.mean(output.submat(center2)).val[2];


        numrects = rects.size();
        drawContours(output, contours, -1, orange);

        //Clear arrays  to stop pipeline from leaking memory
        contours.clear();
        rects.clear();

        //return output
        return output;
    }

    //methods to return values
    public double getRectX() {
        return rectX;
    }

    public double getH() {
        return H;
    }

    public double getS() {
        return S;
    }

    public double getV() {
        return V;
    }

    public int getRects() {
        return numrects;
    }

    public int numcontours() {
        return numcontours;
    }

    public double getRectY() {
        return rectY;
    }
}