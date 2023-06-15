package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.High_pole_max_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.High_pole_min_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Low_pole_max_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Low_pole_min_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Med_pole_max_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Med_pole_min_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.dilate_const;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.erode_const;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.pole_max_H;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.pole_max_S;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.pole_max_V;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.pole_min_H;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.pole_min_S;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.pole_min_V;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionUtils;
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
    private Scalar black = new Scalar(0,0,0);
    private Scalar orange = new Scalar(300, 150, 150);
    private Scalar high = new Scalar(255, 0, 0); //red
    private Scalar med = new Scalar(0, 255, 0); //green

    private Scalar low = new Scalar(0, 0, 255); //blue

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public int numcontours;
    public int numrects;

    private int font = FONT_HERSHEY_COMPLEX;
    private double rectX = 0;
    public double TargetHighrectX = 0;
    private double TargetHighrectY = 0;
    public double TargetMedrectX = 0;
    public double TargetLowrectX = 0;
    private double TargetMedrectY = 0;

    private double TargetLowrectY = 0;

    private double H;

    public double Target_High_Rect_Width = 0;
    public double Target_Med_Rect_Width = 0;

    public double Target_Low_Rect_Width = 0;
    public double Largest_Rect_Width = 0;
    private double S;

    private double V;

    //create rects
    static final Rect center2 = new Rect(new Point(200, 180), new Point(350, 320));
    private double rectY = 0;

    Rect rect = new Rect(new Point(0,0), new Point(50, 50));
    private Rect largestRect;
    private List<Rect> rects = new ArrayList<>();
    //POLE WIDTH SPECIFIC VARIABLES
    private List<Rect> OrderedByWidthrects = new ArrayList<>();

    private List<Rect> OrderedByHeightrects = new ArrayList<>();

    private int HighRect = -1;

    private int LowRect = -1;
    private int MedRect = -1;
    private Rect TargetHighRect;
    private Rect TargetMedRect;

    private Rect TargetLowRect;
    public int rectangles;

    //colour scales for the cone
    public Scalar MIN_THRESH;
    public Scalar MAX_THRESH;

    public Scalar values;


    @Override
    public Mat processFrame(Mat input) {

        MIN_THRESH = new Scalar(pole_min_H, pole_min_S, pole_min_V);
        MAX_THRESH = new Scalar(pole_max_H, pole_max_S, pole_max_V);
        // copy input to output
        input.copyTo(output);

        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        //convert to HSV
        Imgproc.cvtColor(input, modified, COLOR_RGB2HSV);

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

        HighRect = -1;
        MedRect = -1;
        LowRect = -1;
        numcontours = contours.size();

        if(rects.size() > 0) {

            //order the rectangles by width and find the first one that is the expected pole width
            OrderedByWidthrects = VisionUtils.sortRectsByMaxOption(rects.size(), VisionUtils.RECT_OPTION.WIDTH, rects);

            //find the widths expected for a high pole and a medium pole
            for (int i = 0; i < OrderedByWidthrects.size(); i++) {

                if (OrderedByWidthrects.get(i).width < High_pole_max_width && (OrderedByWidthrects.get(i).width > High_pole_min_width)) {
                    HighRect = i;
                }
//                if (OrderedByWidthrects.get(i).width < Med_pole_max_width && (OrderedByWidthrects.get(i).width > Med_pole_min_width)) {
//                    MedRect = i;
//                }
//
//                if (OrderedByWidthrects.get(i).width < Low_pole_max_width && (OrderedByWidthrects.get(i).width > Low_pole_min_width)) {
//                    LowRect = i;
//                }

            }


            if (HighRect > -1) {

                TargetHighRect = OrderedByWidthrects.get(HighRect);
                rectangle(output, TargetHighRect, high, 8);

                Target_High_Rect_Width = TargetHighRect.width;
                TargetHighrectX = TargetHighRect.x + TargetHighRect.width / 2;
                TargetHighrectY = TargetHighRect.y + TargetHighRect.height / 2;

                Imgproc.circle(output, new Point(TargetHighrectX, TargetHighrectY), 10, high, 5);
                Imgproc.putText(output, "High Pole", new Point(TargetHighrectX + Target_High_Rect_Width / 2, TargetHighrectY - 10), FONT_HERSHEY_COMPLEX, 0.5, black, 2);

            }else{
                TargetHighrectX = -1;
                Target_High_Rect_Width = -1;

            }

//            if (MedRect > -1) {
//                TargetMedRect = OrderedByWidthrects.get(MedRect);
//                rectangle(output, TargetMedRect, med, 8);
//
//
//                Target_Med_Rect_Width = TargetMedRect.width;
//                TargetMedrectX = TargetMedRect.x + TargetMedRect.width / 2;
//                TargetMedrectY = TargetMedRect.y + TargetMedRect.height / 2;
//
//                Imgproc.circle(output, new Point(TargetMedrectX, TargetMedrectY), 10, med, 5);
//                Imgproc.putText(output, "Medium Pole", new Point(TargetMedrectX + Target_Med_Rect_Width / 2, TargetMedrectY - 10), FONT_HERSHEY_COMPLEX, 0.5, black, 2);
//
//            }else{
//                TargetMedrectX = -1;
//                Target_Med_Rect_Width = -1;
//
//            }
//
//            if (LowRect > -1) {
//                TargetLowRect = OrderedByWidthrects.get(LowRect);
//                rectangle(output, TargetLowRect, low, 8);
//
//
//                Target_Low_Rect_Width = TargetLowRect.width;
//                TargetLowrectX = TargetLowRect.x + TargetLowRect.width / 2;
//                TargetLowrectY = TargetLowRect.y + TargetLowRect.height / 2;
//
//                Imgproc.circle(output, new Point(TargetLowrectX, TargetLowrectX), 10, med, 5);
//                Imgproc.putText(output, "Medium Pole", new Point(TargetLowrectX + Target_Low_Rect_Width / 2, TargetLowrectY - 10), FONT_HERSHEY_COMPLEX, 0.5, black, 2);
//
//            }else{
//                TargetLowrectX = -1;
//                Target_Low_Rect_Width = -1;
//            }
            //find largest rect and draw a rectangle and mark it with a circle in the center


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
        HighRect = -1;
        MedRect = -1;
        LowRect = -1;
        //return output
        return output;
    }

    //methods to return values
    public double getRectX() {
        return rectX;
    }
    public double getTargetHighrectX() {
        return TargetHighrectX;
    }
    public double getTargetMedrectX() {
        return TargetMedrectX;
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