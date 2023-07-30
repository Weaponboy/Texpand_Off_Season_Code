package org.firstinspires.ftc.teamcode.ConstantsAndSetPoints;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Constants {

    /**Collecting cone constants*/
    public static boolean slides_Power = false;

    public static boolean Slides_Safety_Stop = false;

    public static double Collect_Cone_Distance = 70;

    public static double Auto_Collect_Cone_Distance = 50;

    public static double slides_slow_point = 180;

    public static boolean SlowPoint = false;

    public static CurrentUnit ampere;

    public static boolean collecting_cone = false;

    public static Orientation heading;


    /**Slides power*/

    public static double Collection_slow_power = -0.3;

    public static double Collection_Slides_Max_Speed = -0.8;

    public static double Delivery_Slides_Max_Speed = 1;

    public static double Delivery_Slides_Max_Speed_Reverse = -0.9;

    public static double Slides_Reverse_Max_Speed = 0.6;

    /**Top Pivot PID*/

    public static double ticks_in_degrees = 1120 / 180.0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    public static double pivot_f = 0;

    public static int Pivot_Target = 0;

    public static int Pivot_Current_Position = 0;

    public static double Top_Pivot_PID = 0;

    public static double Pivot_FF = 0;

    public static double Pivot_Power = 0;

    /**Collecting elapsed time*/

    public static double Start_Time = 0;

    public static double Current_Time = 0;

    public static double Time_Difference = 0;

    /**Camera offset center to account for physical camera offset*/
    //left offset
    public static double Offset_Left = 309;

    //right offset
    public static double Offset_Right = 329;

    public static double TargetPixels = 0;

    //convert camera pixels to cm's
    public static double deltaServoPosition;

    public static double servoPosition;

    public static double ConversionPixelstoServoPosition = 3200;

    //Camera real center
    public static double CenterOfScreen = 320;

    /**Drive constants*/

    public static double denominator;

    public static double botHeading;

    //toggle speed
    public static double slow1 = 0.4;

    public static double vertical;
    public static double horizontal;
    public static double pivot;

    public static double throttle = 0.6;

    public static double brake = 0.5;

    /**Slides constants*/

    public static boolean lowering = false;

    public static boolean Full_Cycle_Toggle = false;

    /**Nest check*/

    public static double Nest_Check_Blue = 1500;

    /**Cone found variables*/
    public static double CurrentDraw = 0;
    public static double OldCurrent = 0;
    public static boolean CurrentDrawSpike = false;

    public static double counterfornest = 0;

    public static boolean conefound = false;

    public static boolean conefound_Noslides = false;

    public static boolean Trigger_Collect = false;

    public static boolean abort = false;

    public static boolean Time = false;

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

    /**Odometry*/

    public static PIDFController drivePID;
    public static PIDFController strafePID;
    public static PIDFController PivotPID;

    public static double Xdist = 0;
    public static double Ydist = 0;

    public static double rotdist = 0;

    public static double XdistForStop = 0;
    public static double YdistForStop = 0;

    public static double rotdistForStop = 0;

    public static double RRXdist = 0;
    public static double RRYdist = 0;
    public static double Horizontal = 0;
    public static double Vertical = 0;

    public static double Pivot = 0;

    public static double ConvertedHeading = 0;

    public static boolean Nest_Check = false;

}