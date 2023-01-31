package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class VisionDash {

    public static Point target = new Point(0, 0);

    public static boolean debugMode;

    public static int blur = 75;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static double area_cutoff = 500;


    public static int red_min_Y = 110;
    public static int red_min_Cr = 180;
    public static int red_min_Cb = 105;

    public static int red_max_Y = 170;
    public static int red_max_Cr = 230;
    public static int red_max_Cb = 130;

    public static int blue_min_Y = 110;
    public static int blue_min_Cr = 180;
    public static int blue_min_Cb = 105;

    public static int blue_max_Y = 170;
    public static int blue_max_Cr = 230;
    public static int blue_max_Cb = 130;

    //Cone color
    public static int blue_min_H = 135;
    public static int blue_min_S = 0;
    public static int blue_min_V = 0;

    public static int blue_max_H = 165;
    public static int blue_max_S = 100;
    public static int blue_max_V = 100;

    public static int max_Cb = 255;
    public static int max_Cr = 255;
    public static int max_Y = 38;

    public static int min_Cb = 100;
    public static int min_Cr = 40;
    public static int min_Y = 10;


//    public static int max_Cb = 100;
//    public static int max_Cr = 165;
//    public static int max_Y = 255;
//
//    public static int min_Cb = 70;
//    public static int min_Cr = 135;
//    public static int min_Y = 100;


    public static double blackMultiplier = 2;



    public static double[] YCbCrReadout = {0, 0, 0};

}
