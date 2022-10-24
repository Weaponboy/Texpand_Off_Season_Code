package org.firstinspires.ftc.teamcode;

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
public class OpenCVPipelinetest extends LinearOpMode {

    private Josh_Test_Pipeline detect = new Josh_Test_Pipeline(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(detect);
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

            }
        });

//        switch (detect.getLocation()){
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

class Josh_Test_Pipeline_indent extends OpenCvPipeline {

    Telemetry telemetry;
    public Josh_Test_Pipeline_indent(Telemetry t) {
        telemetry = t;
    }

    //create matrix's
    Mat workingMatrix = new Mat();
    Mat Purple = new Mat();
    Mat Yellow = new Mat();
    Mat Green = new Mat();

    // region of interest
    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));

    //color threshold
    static double COLOR_THRESHOLD = 0.2;

    //store location
//    public enum Location {
//        purple,
//        green,
//        yellow,
//        not_found
//    }

//    private Location location;

    @Override
    public Mat processFrame(Mat input) {

        //copy to all matrix's
        input.copyTo(workingMatrix);
        input.copyTo(Purple);
        input.copyTo(Yellow);
        input.copyTo(Green);

        //color scales
        Imgproc.cvtColor(Purple, Purple, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Green, Green, Imgproc.COLOR_RGB2HSV);

        Scalar purpleup = new Scalar(290/2, 100, 99);
        Scalar purpledown = new Scalar(240/2, 30, 20);
        Scalar greenup = new Scalar(140/2, 100, 99);
        Scalar greendown = new Scalar(95/2, 32, 20);
        Scalar yellowup = new Scalar(80/2, 100, 99);
        Scalar yellowdown = new Scalar(10/2, 35, 20);

        // check for colors on the matrix's
        Core.inRange(Purple, purpledown, purpleup , Purple);
        Core.inRange(Green, greendown, greenup, Green);
        Core.inRange(Yellow, yellowdown, yellowup, Yellow);

        Purple.submat(center);
        Green.submat(center);
        Yellow.submat(center);

        double PURPLE = Math.round(Core.mean(Purple).val[2]);
        double GREEN = Math.round(Core.mean(Green).val[2]);
        double YELLOW = Math.round(Core.mean(Yellow).val[2]);

        Scalar greenrect = new Scalar(130/2, 100, 50);
        Scalar yellowrect = new Scalar(204/2, 100, 0);
        Scalar purplerect = new Scalar(274/2, 100, 99);

        telemetry.addData("purple raw value", (int) Core.mean(Purple).val[2]);
        telemetry.addData("yellow raw value", (int) Core.mean(Yellow).val[2]);
        telemetry.addData("green raw value", (int) Core.mean(Green).val[2]);


        if (YELLOW > COLOR_THRESHOLD){
////            location = Location.yellow;
            Imgproc.rectangle(workingMatrix, center, yellowrect, 1);
            telemetry.addData("Color", "Yellow");
        }else if (GREEN > COLOR_THRESHOLD){
////            location = Location.green;
            Imgproc.rectangle(workingMatrix, center, greenrect, 1);
            telemetry.addData("Color", "Green");
        }else if (PURPLE > COLOR_THRESHOLD){
////            location = Location.purple;
            Imgproc.rectangle(workingMatrix, center, purplerect, 1);
            telemetry.addData("Color", "Purple");
        }
////        else{
////            location = Location.not_found;
////            return input;
////        }
        telemetry.update();


        return workingMatrix;

    }

//    public Location getLocation(){
//        return location;
//    }
}


