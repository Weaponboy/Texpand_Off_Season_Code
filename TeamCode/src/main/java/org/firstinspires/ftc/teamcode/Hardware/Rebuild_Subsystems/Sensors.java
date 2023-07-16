package org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Vision.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class Sensors {

    HardwareMap hardwareMap;

    public DistanceSensor Collect_Cone;

    public ColorSensor Nest_Check;

    public OpenCvWebcam BackWeb;

    public Pole_Pipe pole;

    public void init(HardwareMap Hmap, boolean Stream){

        hardwareMap = Hmap;

        pole = new Pole_Pipe();


        Nest_Check = hardwareMap.get(ColorSensor.class, "colour");

        Collect_Cone = hardwareMap.get(DistanceSensor.class, "sensor_range");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Backcam"));

        if (Stream){
            BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {


                    BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                    BackWeb.getExposureControl().setExposure(80, TimeUnit.MILLISECONDS);

                    BackWeb.getGainControl().setGain(60);

                    FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                    BackWeb.getFocusControl().setMode(focusmode);

                    if (focusmode == FocusControl.Mode.Fixed){
                        BackWeb.getFocusControl().setFocusLength(450);
                    }

                    BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }
                @Override
                public void onError(int errorCode) { }
            });

            BackWeb.setPipeline(pole);

            FtcDashboard.getInstance().startCameraStream(BackWeb,30);
        }

    }

    public void Start_Cam(){
        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {


                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                BackWeb.getExposureControl().setExposure(25, TimeUnit.MILLISECONDS);

                BackWeb.getGainControl().setGain(1);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                BackWeb.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    BackWeb.getFocusControl().setFocusLength(450);
                }

                BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });

        BackWeb.setPipeline(pole);


        FtcDashboard.getInstance().startCameraStream(BackWeb,30);
    }

}
