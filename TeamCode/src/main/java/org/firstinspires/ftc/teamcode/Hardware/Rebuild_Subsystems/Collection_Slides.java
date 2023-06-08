package org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collection_Slides {

    HardwareMap hardwareMap;

    public DcMotorEx Extend;

    public void init(HardwareMap Hmap){

        hardwareMap = Hmap;

        Extend = hardwareMap.get(DcMotorEx.class, "Extend");

        Extend.setDirection(DcMotorSimple.Direction.REVERSE);

        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Collect_RunToPosition (int setpoint, double power){

        Extend.setTargetPosition(setpoint);

        Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Extend.setPower(power);
    }
}
