package org.firstinspires.ftc.teamcode.Vision.Pole_Alinement;

//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class Pole_Vision extends OpMode {
    Drivetrain drive = new Drivetrain();

    private ElapsedTime runtime = new ElapsedTime();
    Pole_Pipe Pole;

    private DistanceSensor Back_Distance;

    private OpenCvCamera webcam;

    private  double poleDistance = 0;

    private  double TargetPoleDistance = 32;

    private  double TravelDistance = 0;

    private  int Loop = 0;

    public double Distance_To_Travel;

    public double ConversionPixelstoCm = 20;//need to tune this

    public double CenterOfScreen = 320;

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

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(Pole);
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });
        // Set the pipeline depending on id
        Texpandcamera.setPipeline(Pole);

    }


    @Override
    public void init_loop() {

        drive.init(hardwareMap);

        telemetry.addData("H:", Pole.getH());
        telemetry.addData("S:", Pole.getS());
        telemetry.addData("V:", Pole.getV());
        telemetry.update();
    }



    public void start() {

    }


    @Override
    public void loop() {

//        rectPositionFromLeft = Pole.getRectX();
//
//        Distance_To_Travel = rectPositionFromLeft - CenterOfScreen;
//
//        Distance_To_Travel = Distance_To_Travel / 35;

        if (Distance_To_Travel > 0){
            drive.TurnDegreesLeft(Distance_To_Travel);
            drive.stopMotors();
            Distance_To_Travel = 0;
        }else if (Distance_To_Travel < 0){
            drive.TurnDegrees(Distance_To_Travel);
            drive.stopMotors();
            Distance_To_Travel = 0;
        }


//        if(Distance_To_Travel == 0){
//            poleDistance = Back_Distance.getDistance(DistanceUnit.CM);
//
//            TravelDistance = poleDistance - TargetPoleDistance;
//            while(Loop < 1){
//                Loop++;
//                if (TravelDistance > 0){
//                    drive.DriveDistanceLongReverse(TravelDistance, 0.5);
//                    drive.stopMotors();
//                    TravelDistance = 0;
//                }else if (TravelDistance < 0) {
//                    drive.DriveDistanceLong(-TravelDistance, 0.5);
//                    drive.stopMotors();
//                    TravelDistance = 0;
//                }
//                Loop++;
//            }
//
//        }

        telemetry.addData("Target CM", Distance_To_Travel);
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
