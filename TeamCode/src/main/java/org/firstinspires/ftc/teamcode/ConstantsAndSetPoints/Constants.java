package org.firstinspires.ftc.teamcode.ConstantsAndSetPoints;

public class Constants {

    /**Camera offset center to account for physical camera offset*/
    //left offset
    public double Offset_Left = 309;

    //right offset
    public double Offset_Right = 329;

    //convert camera pixels to cm's
    public double ConversionPixelstoCm = 20;

    //Camera real center
    public double CenterOfScreen = 320;

    /**Drive speed constants*/
    //throttle speed
    public static boolean SlowPoint = false;

    //second toggle speed
    public double slow = 0.4;

    //toggle speed
    public double slow1 = 0.4;

    /**Slides constants*/

    public boolean lowering = false;

    public double reverseSlidesPower = 0.005;

    /**Cone found variables*/

    public boolean conefound = false;

    public boolean conefoundcycle;

    /**toggle gripper and pivot positions*/

    public double Destack_position = 0;

    //toggle the top pivot position
    public int Toppos = 0;

    //toggle the top pivot position
    public int Toppos2 = 0;

    //toggle destacker position
    public int stakerpos = 0;

    //used for toggling the pole vision on and off
    private int TopposP = 0;

    /**Vision alignment constants*/

    //motor power for cone stack alignment
    public double Cone_power;

    //power for vision alignment
    public double power;

    //For toggling the pole vision on and off
    public boolean PoleAlignmnet = true;

    //variable for holding the cone stack/pole position in the camera stream
    public double rectPositionFromLeft = 0;


}