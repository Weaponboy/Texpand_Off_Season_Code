package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class OLdPipeline extends LinearOpMode {

    Threshold_Pipeline thresholdPipe = new Threshold_Pipeline();
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(new Threshold_Pipeline(telemetry));
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {


            }
        });

//        switch (Threshold_Pipe.Location.getLocation()){
//            case green:
//                telemetry.addData("Position", "Green");
//                break;
//            case purple:
//                telemetry.addData("Position", "Purple");
//                break;
//            case yellow:
//                 telemetry.addData("Position", "Yellow");
//                 break;
//                 default:
//        }
//        telemetry.update();


        waitForStart();

        Texpandcamera.closeCameraDevice();
    }
}

class Threshold_Pipe extends OpenCvPipeline {


    private Location location;

    public enum Location{
        Left_pos,
        Right_pos,
        Middle_pos;

        
    }
    double color = 0;
    final double Threshold = 0.01;
    Mat workingmatrix = new Mat();
    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));
    Telemetry telemetry;
    public Threshold_Pipe(Telemetry t) {
        telemetry = t;
    }

    public Threshold_Pipe() {

    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingmatrix);


        //color scales

        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV_FULL);

        Scalar bottom = new Scalar(355, 100, 100);
        Scalar top = new Scalar(0, 0, 0);
        Scalar redup = new Scalar(60, 100, 100);
        Scalar reddown = new Scalar(0, 40, 40);


        // check for colors on the matrix's

        //Core.inRange(workingmatrix, reddown, redup , workingmatrix);

        //workingmatrix.submat(center);



        //color = Math.round(Core.mean(workingmatrix).val[0] / 255);





//        workingmatrix.release();
        telemetry.addData("HUE", Math.round(Core.mean(workingmatrix).val[0]) );
        telemetry.addData("SATURATION", Math.round(Core.mean(workingmatrix).val[1]) );
        telemetry.addData("VALUE", Math.round(Core.mean(workingmatrix).val[2]) );




        if (Core.mean(workingmatrix).val[0] > 30 && Core.mean(workingmatrix).val[0] < 110){
            telemetry.addData("Colour", "Yellow");
            location = Location.Middle_pos;

        }else if(Core.mean(workingmatrix).val[0] > 110 && Core.mean(workingmatrix).val[0] < 148){
            telemetry.addData("Colour", "Blue");

        }else if(Core.mean(workingmatrix).val[0] > 150 && Core.mean(workingmatrix).val[0] < 250){
            telemetry.addData("Colour", "Red");

        }
        telemetry.update();
        return input;
    }

    public Location getLocation() {
       return location;
    }
}