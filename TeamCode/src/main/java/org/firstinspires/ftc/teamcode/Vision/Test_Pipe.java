package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.High_pole_max_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.High_pole_min_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Low_pole_max_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Low_pole_min_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Med_pole_max_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.Med_pole_min_width;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.dilate_const;
import static org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionDash.erode_const;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import org.firstinspires.ftc.teamcode.Vision.Vision_Utils.VisionUtils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Test_Pipe extends OpenCvPipeline {

    Mat moding = new Mat();

    Rect center = new Rect(new Point(100, 100), new Point(200, 200));

    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private List<Rect> OrderedByWidthrects = new ArrayList<>();

    private List<Rect> rects = new ArrayList<>();

    private Mat hierarchy = new Mat();

    public int numcontours;

    private Rect largestRect;

    private int highRect;

    private int MedRect;

    private int LowRect;

    private Rect TargetHighRect;

    private Scalar high = new Scalar(255, 0, 0); //red

    public double TargetHighrectX = 0;

    private double TargetHighrectY = 0;

    public double Target_High_Rect_Width = 0;

    private Scalar black = new Scalar(0,0,0);

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(moding);

        Scalar yellowlower = new Scalar(15, 100, 60);
        Scalar yellowupper = new Scalar(40, 255, 255);

        Imgproc.cvtColor(input, moding, COLOR_RGB2HSV);

//        Core.mean(output.submat(center));

        inRange(moding, yellowlower, yellowupper, moding);

        erode(moding, moding, new Mat(erode_const, erode_const, CV_8U));

        dilate(moding, moding, new Mat(dilate_const, dilate_const, CV_8U));

        findContours(moding, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        if (rects.size() > 0){

            OrderedByWidthrects = VisionUtils.sortRectsByMaxOption(rects.size(), VisionUtils.RECT_OPTION.WIDTH, rects);

            //find the widths expected for a high pole and a medium pole
            for (int i = 0; i < OrderedByWidthrects.size(); i++) {

                if (OrderedByWidthrects.get(i).width < High_pole_max_width && (OrderedByWidthrects.get(i).width > High_pole_min_width)) {
                    highRect = i;
                }

                if (OrderedByWidthrects.get(i).width < Med_pole_max_width && (OrderedByWidthrects.get(i).width > Med_pole_min_width)) {
                    MedRect = i;
                }

                if (OrderedByWidthrects.get(i).width < Low_pole_max_width && (OrderedByWidthrects.get(i).width > Low_pole_min_width)) {
                    LowRect = i;
                }

            }

            if (highRect > -1) {
                TargetHighRect = OrderedByWidthrects.get(highRect);
                rectangle(input, TargetHighRect, high, 8);

                Target_High_Rect_Width = TargetHighRect.width;
                TargetHighrectX = TargetHighRect.x + TargetHighRect.width / 2;
                TargetHighrectY = TargetHighRect.y + TargetHighRect.height / 2;

                Imgproc.circle(input, new Point(TargetHighrectX, TargetHighrectY), 10, high, 5);
                Imgproc.putText(input, "High Pole", new Point(TargetHighrectX + Target_High_Rect_Width / 2, TargetHighrectY - 10), FONT_HERSHEY_COMPLEX, 0.5, black, 2);

            }else{
                TargetHighrectX = -1;
                Target_High_Rect_Width = -1;
            }

        }

        return input;
    }

    public double getHighTarget(){
        return TargetHighrectX;
    }

}
