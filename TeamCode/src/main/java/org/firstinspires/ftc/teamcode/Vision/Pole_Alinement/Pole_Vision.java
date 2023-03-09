package org.firstinspires.ftc.teamcode.Vision.Pole_Alinement;

//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Vision_From_Collin.VisionDash;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;


@Autonomous
@Config
public class Pole_Vision extends OpMode {
    Drivetrain drive = new Drivetrain();

    private ElapsedTime runtime = new ElapsedTime();
    Pole_Pipe Pole;

    private DistanceSensor Back_Distance;

    private OpenCvCamera webcam;
    public BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    private double power;

    private  double poleDistance = 0;

    private  double TargetPoleDistance = 32;

    private  double TravelDistance = 0;

    public static int Loop = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Orientation yawAngle;
    public double Degrees_To_Turn;

    public double ConversionPixelstoCm = 20;//need to tune this

    public static double CenterOfScreen;

    public double rectPositionFromLeft = 0;


    public void init() {

        drive.init(hardwareMap);

//        drive.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pole = new Pole_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Backcam");

        OpenCvWebcam Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(Pole);
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Texpandcamera.getExposureControl().setExposure(20, TimeUnit.MILLISECONDS);

                Texpandcamera.getGainControl().setGain(50);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                Texpandcamera.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    Texpandcamera.getFocusControl().setFocusLength(300);
                }

                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });

        // Set the pipeline depending on id
        Texpandcamera.setPipeline(Pole);

        FtcDashboard.getInstance().startCameraStream(Texpandcamera,30);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    @Override
    public void init_loop() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CenterOfScreen = 320;
        drive.init(hardwareMap);


        rectPositionFromLeft = Pole.TargetHighrectX;

        Degrees_To_Turn = rectPositionFromLeft - CenterOfScreen;

        Degrees_To_Turn = Degrees_To_Turn / 20;

        //Telemetry to be displayed during init_loop()

        telemetry.addData("H max", VisionDash.pole_max_H);
        telemetry.addData("H min", VisionDash.pole_min_H);
        telemetry.addData("S", Pole.getS());
        telemetry.addData("V", Pole.getV());
        telemetry.addData("H", Pole.getH());
        telemetry.addData("con", Pole.numcontours());
        telemetry.addData("rects", Pole.getRects());
        telemetry.addData("Ordered rectangles", Pole.rectangles);
        telemetry.addData("Target cm", Degrees_To_Turn);
        telemetry.addData("Largestrect Width", Pole.Largest_Rect_Width);
        telemetry.addData("Targetrect High Width", Pole.Target_High_Rect_Width);
        telemetry.addData("Targetrect Med Width", Pole.Target_Med_Rect_Width);
        telemetry.update();
    }



    public void start() {

    }


    @Override
    public void loop() {
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        power = 0.35;
        drive.DriveEncoders();
        if(Degrees_To_Turn != 0) {
            drive.TurnToHeading(yawAngle.firstAngle + Degrees_To_Turn, 0.3);
            Degrees_To_Turn = 0;
        }

        //Telemetry to be displayed during loop()
        telemetry.addData("rect X", Pole.getRectX());
        telemetry.addData("rect Y", Pole.getRectY());
        telemetry.addData("Target Heading", Degrees_To_Turn + yawAngle.firstAngle);
        telemetry.addData("Heading", yawAngle.firstAngle);

        telemetry.update();
    }

    //Telemetry to be displayed during init_loop()
    private void initTelemetry(){
        telemetry.addData("rect X", Pole.getRectX());
        telemetry.addData("rect Y", Pole.getRectY());
        telemetry.addData("num contours", Pole.numcontours);
        telemetry.addData("num rects", Pole.numrects);
        telemetry.addData("HSV values", Pole.values);
        telemetry.update();
    }

    //Telemetry to be displayed during loop()

    private void loopTelemetry(){

    }
}
