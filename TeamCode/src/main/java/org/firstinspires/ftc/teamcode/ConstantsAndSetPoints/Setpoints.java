package org.firstinspires.ftc.teamcode.ConstantsAndSetPoints;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Setpoints {

    /** Slide Set Points*/

    //High pole set point
    public static int High_Pole_Cycle = 640;

    public static int High_Pole_Cycle_while = 600;

    //Top pole set point
    public static int High_Pole_Driver = 740;

    public static int High_Pole_Auto = 920;

    //Medium pole set point
    public static int Medium_Pole = 250;

    //Collection slides full extension safety stop
    public static double Stop_Point = -550;

    /**Base gripper positions*/
    //open
    public static double Base_Gripper_Open = 0.4;

    //closed
    public static double Base_Gripper_Closed = 0;

    //Half open
    public static double Base_Gripper_Half_Open = 0.35;

    /**Top gripper positions*/
    //open
    public static double Top_Gripper_Open = 0.3;

    //closed
    public static double Top_Gripper_Closed = 0;

    //wide open
    public static double Top_Gripper_Open_Wide = 0.45;

    /**Top Pivot positions*/

    //Get cone from nest
    public static int Top_Pivot_Nest_Position = 0;

    //waiting for cone
    public static int Top_Pivot_Waiting_For_Cone = 200;

    public static int Drop_Cone_Cycle = 900;

    //Ready for drop
    public static int Top_Pivot_Almost_Drop = 750;

    //Position to drop the cone of once the slides have reached position
    public static int Top_Pivot_Deliver = 950;

    /**Top Turn positions*/
    public static double Top_Turn_Middle = 0.5;

    public static double Top_Turn_Left = 0.2;

    public static double Top_Turn_Right = 0.8;

    /**Base Pivot positions*/

    //ready to collect a cone
    public static double Base_Pivot_Collect = 0.01;

    //position to put the cone in the nest
    public static double Base_Pivot_Flip = 0.58;

    //Pivot all the way inside the bot
    public static double Base_Pivot_Out_Way = 0.95;

    public static double Base_Pivot_Transfer_Pos = 0.75;

    /**Destacker positions*/

    //the top cone on the stack
    public static double De_Pos_1 = 0.36;

    //second cone from the top of the cone stack
    public static double De_Pos_2 = 0.38;

    //middle cone on the stack
    public static double De_Pos_3 = 0.5;

    //second from the bottom cone on the stack
    public static double De_Pos_4 = 0.72;

    //normal cone collection position
    public static double De_Pos_5 = 1;
}
