package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bottom_Gripper_Assembly {

    public Servo Destacker = null;
    public Servo Base_Gripper = null;
    public Servo Base_Pivot = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    private int stakerpos = 0;
    private int Bpivotpos = 0;
    private int BGripperpos = 0;


    public Bottom_Gripper_Assembly() { }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        Base_Gripper = hardwareMap.get(Servo.class,"Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class,"Base_Pivot");
        Destacker = hardwareMap.get(Servo.class,"Destacker");

        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);

        Base_Pivot.setPosition(0.35);
        Base_Gripper.setPosition(0.45);
        Destacker.setPosition(0.2);
    }

    public void destacker1(){
        stakerpos = stakerpos + 1;

        if(stakerpos == 1){
            Destacker.setPosition(1);
            Base_Pivot.setPosition(0);
        }else if(stakerpos == 2){
            Destacker.setPosition(0.7);
            Base_Pivot.setPosition(0.2);
        }else if(stakerpos == 3) {
            Destacker.setPosition(0.2);
            Base_Pivot.setPosition(0.35);
        }
        if(stakerpos > 3){
            stakerpos = 0;
        }

    }

    public void Base_Pivot(){
        Bpivotpos = Bpivotpos + 1;

        if(Bpivotpos == 1 && Base_Pivot.getPosition() != 0.35 ){
            Base_Pivot.setPosition(0.35); //close base gripper if it is open
        }else if(Bpivotpos == 2 && Base_Pivot.getPosition() != 1){
            Base_Pivot.setPosition(1); //open base gripper if it is closed

        }
        if(Bpivotpos > 2){
            Bpivotpos = 0;
        }

    }

    public void Base_Gripper(){
        BGripperpos = BGripperpos + 1;

        if(BGripperpos == 1 && Base_Gripper.getPosition() == 0){
            Base_Gripper.setPosition(0.45); //open base gripper if it is closed
        }else if(BGripperpos == 2 && Base_Gripper.getPosition() > 0){
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }
        if(BGripperpos > 2){
            BGripperpos = 0;
        }

    }
}
