package org.firstinspires.ftc.teamcode.Hardware.Sub_Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bottom_Gripper_Assembly {

    public Servo Destacker_Left = null;
    public Servo Destacker_Right = null;
    public Servo Base_Gripper = null;
    public Servo Base_Pivot = null;

    private double Destack_position = 0;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    private int stakerpos = 0;
    private int Bpivotpos = 0;
    private int BGripperpos = 0;

    private double Base_Pivot_Collect = 0.08;

    private double Base_Pivot_Flip = 0.78;

    private double Base_Pivot_Out_Way = 1;


    public Bottom_Gripper_Assembly() {}

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        Base_Gripper = hardwareMap.get(Servo.class,"Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class,"Base_Pivot");
        Destacker_Right = hardwareMap.get(Servo.class,"Destacker Right");
        Destacker_Left = hardwareMap.get(Servo.class,"Destacker Left");

        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Destacker_Left.setDirection(Servo.Direction.REVERSE);

        Base_Gripper.setPosition(0.4);

        Base_Pivot.setPosition(0.8);
    }

    public void destacker1(){
        stakerpos = stakerpos + 1;

        if(stakerpos == 1){
            Destack_position = 1;
        } else if(stakerpos == 2) {
            Destack_position = 0.7;
        } else if(stakerpos == 3) {
            Destack_position = 0.65;
        } else if(stakerpos == 4) {
            Destack_position = 0.55;
        } else if(stakerpos == 5) {
            Destack_position = 0.47;
        }
        if(stakerpos > 5){
            stakerpos = 0;
        }
        Destacker_Left.setPosition(Destack_position);
        Destacker_Right.setPosition(Destack_position);
    }

    public void Base_Pivot(){
        Bpivotpos = Bpivotpos + 1;

        if(Bpivotpos == 1 &&  Base_Pivot.getPosition() != 0.35 ){
            Base_Pivot.setPosition(1); //close base gripper if it is open
            stakerpos = 1;
        }else if(Bpivotpos == 2 &&  Base_Pivot.getPosition() != 1){
            Base_Pivot.setPosition(0); //open base gripper if it is closed

        }
        if(Bpivotpos > 2){
            Bpivotpos = 0;
        }

    }

    public void Base_Gripper(){
        BGripperpos = BGripperpos + 1;

        if(BGripperpos == 1 &&  Base_Gripper.getPosition() == 0){
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
        }else if(BGripperpos == 2 &&  Base_Gripper.getPosition() > 0){
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }
        if(BGripperpos > 2){
            BGripperpos = 0;
        }

    }


}
