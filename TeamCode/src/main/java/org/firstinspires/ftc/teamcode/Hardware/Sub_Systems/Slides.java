package org.firstinspires.ftc.teamcode.Hardware.Sub_Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Red_Cone_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class Slides{

    Telemetry telemetry;
    public Slides(Telemetry t){telemetry = t;}
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public OpenCvWebcam FrontWeb;
    public OpenCvWebcam BackWeb;

    private DistanceSensor Back_Distance;

    public DcMotor Odo_raise  = null;
    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;
    public DcMotor Extend = null;

    public DistanceSensor sensorRange;

    public ColorSensor colour = null;

    public Blue_Cone_Pipe Cone_Pipeline_Blue;

    public Red_Cone_Pipe Cone_Pipeline_Red;

    public Pole_Pipe Pole;

    public Gamepad.RumbleEffect customRumbleEffect;

//    Top_gripper top = new Top_gripper();
//    Bottom_Gripper_Assembly B_grip = new Bottom_Gripper_Assembly();

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    private boolean lowering = false;
    private boolean conefound = false;
    private boolean extending = false;


    public Slides() { }


    public void init(HardwareMap hwMap, double vision) {

        hardwareMap = hwMap;

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        Back_Distance = hardwareMap.get(DistanceSensor.class, "Back distance");

        Odo_raise = hardwareMap.get(DcMotor.class, "Odo_motor");

        colour = hardwareMap.get(ColorSensor.class, "colour");

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        Extend = hardwareMap.get(DcMotor.class, "Extend");

        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Odo_raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Odo_raise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        Back_Distance.resetDeviceConfigurationForOpMode();

        Pole = new Pole_Pipe();

        Cone_Pipeline_Blue = new Blue_Cone_Pipe();

        Cone_Pipeline_Red = new Red_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        FrontWeb  = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);

        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Backcam"), viewportContainerIds[1]);

        FrontWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                if (vision == 1){
                    FrontWeb.setPipeline(Cone_Pipeline_Blue);
                } else if (vision == 2) {
                    FrontWeb.setPipeline(Cone_Pipeline_Red);
                }

                FrontWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                FrontWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);

                FrontWeb.getGainControl().setGain(1);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                FrontWeb.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    FrontWeb.getFocusControl().setFocusLength(450);
                }

                FrontWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
        if (vision == 1){
            FrontWeb.setPipeline(Cone_Pipeline_Blue);
        } else if (vision == 2) {
            FrontWeb.setPipeline(Cone_Pipeline_Red);
        }


        BackWeb.setPipeline(Pole);
        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {


                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                BackWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);

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
        BackWeb.setPipeline(Pole);
        FtcDashboard.getInstance().startCameraStream(BackWeb,30);
        // Set the pipeline depending on id


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

}
