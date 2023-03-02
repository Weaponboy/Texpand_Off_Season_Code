package org.firstinspires.ftc.teamcode.ConstantsAndSetPoints;

public class Setpoints {

    /** Slide Set Points*/

    //Top pole set point
    public int Top_Pole = 1800;

    //Medium pole set point
    public int Medium_Pole = 900;

    //Collection slides full extension safety stop
    public double Stop_Point = -900;

    /**Base gripper positions*/
    //open
    public double Base_Gripper_Open = 0.4;

    //closed
    public double Base_Gripper_Closed = 0;

    //Half open
    public double Base_Gripper_Half_Open = 0.35;

    /**Top gripper positions*/
    //open
    public double Top_Gripper_Open = 0.3;

    //closed
    public double Top_Gripper_Closed = 0;

    //wide open
    public double Top_Gripper_Open_Wide = 0.34;

    /**Top Pivot positions*/

    //Get cone from nest
    public double Top_Pivot_Nest_Position = 1;

    //Position for after dropping off a cone
    public double Top_Pivot_Just_Dropped = 0.1;

    //waiting for cone
    public double Top_Pivot_Waiting_For_Cone = 0.6;

    //Pull the gripper back after the drop off
    public double Top_Pivot_After_Delivery = 0.4;

    //Ready for drop
    public double Top_Pivot_Almost_Drop = 0.3;

    //Position to drop the cone of once the slides have reached position
    public double Top_Pivot_Deliver = 0;

    /**Base Pivot positions*/

    //ready to collect a cone
    public double Base_Pivot_Collect = 0.08;

    //Position for the cone alignment vision
    public double Base_Pivot_Cone_Alignment = 0.7;

    //position to put the cone in the nest
    public double Base_Pivot_Flip = 0.82;

    //Pivot all the way inside the bot
    public double Base_Pivot_Out_Way = 1;


    /**Destacker positions*/

    //the top cone on the stack
    public double De_Pos_1 = 0.0;

    //second cone from the top of the cone stack
    public double De_Pos_2 = 0.12;

    //middle cone on the stack
    public double De_Pos_3 = 0.45;

    //second from the bottom cone on the stack
    public double De_Pos_4 = 0.54;

    //normal cone collection position
    public double De_Pos_5 = 0.72;
}
