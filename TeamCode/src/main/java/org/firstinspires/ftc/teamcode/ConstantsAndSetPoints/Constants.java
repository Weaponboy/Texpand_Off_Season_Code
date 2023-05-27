package org.firstinspires.ftc.teamcode.ConstantsAndSetPoints;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Constants {

    /**Top Pivot Power*/

    public static double Top_Pivot_Forward = 0.05;

    public static double Top_Pivot_Reverse = -0.05;

    public static boolean slides_Power = false;

    public static double Odostart = 0;

    public static boolean Slides_Safety_Stop = false;

    public static double Collect_Cone_Distance = 70;

    public static double slides_slow_point = 200;

    public static double slow_power = -0.35;

    public static double Slides_Max_Speed = -0.8;

    public static double Slides_Reverse_Max_Speed = 0.6;

    public static double ticks_in_degrees = 1120 / 180.0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    public static double pivot_f = 0;

    public static double vertical;
    public static double horizontal;
    public static double pivot;

    public static double throttle = 0.5;

    public static double brake = 0.5;

    public static CurrentUnit ampere = null;

    public static int Pivot_Target = 0;

    public static int Pivot_Current_Position = 0;

    public static boolean collecting_cone = false;

    public static double Top_Pivot_PID = 0;

    public static double Pivot_FF = 0;

    public static double Pivot_Power = 0;

    /**Camera offset center to account for physical camera offset*/
    //left offset
    public static double Offset_Left = 309;

    //right offset
    public static double Offset_Right = 329;

    //convert camera pixels to cm's
    public static double ConversionPixelstoCm = 20;

    //Camera real center
    public static double CenterOfScreen = 320;

    /**Drive speed constants*/
    //throttle speed
    public static boolean SlowPoint = false;

    //second toggle speed
    public static double slow = 0.4;

    //toggle speed
    public static double slow1 = 0.4;

    /**Slides constants*/

    public static boolean lowering = false;

    public static double reverseSlidesPower = 0.005;

    /**Cone found variables*/

    public static boolean conefound = false;

    public static boolean conefoundcycle;

    /**toggle gripper and pivot positions*/

    public static double Destack_position = 0;

    //toggle the top pivot position
    public static int Toppos = 0;

    //toggle the top pivot position
    public static int Toppos2 = 0;

    //toggle destacker position
    public static int stakerpos = 0;

    //used for toggling the pole vision on and off
    private static int TopposP = 0;

    /**Vision alignment constants*/

    //motor power for cone stack alignment
    public static double Cone_power;

    //power for vision alignment
    public static double power;

    //For toggling the pole vision on and off
    public static boolean PoleAlignmnet = true;

    //variable for holding the cone stack/pole position in the camera stream
    public static double rectPositionFromLeft = 0;


}