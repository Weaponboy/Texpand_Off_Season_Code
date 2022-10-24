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
public class OpenCVThreshold extends LinearOpMode {

    threshold_Pipeline thresholdPipe = new threshold_Pipeline();
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(new threshold_Pipeline(telemetry));
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {


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

class threshold_Pipeline extends OpenCvPipeline {
    double color = 0;
    final double Threshold = 0.01;
    Mat workingmatrix = new Mat();
    Mat Purple = new Mat();
    Mat Yellow = new Mat();
    Mat Green = new Mat();
    static final Rect center = new Rect(new Point(100, 100), new Point(550, 350));
    Telemetry telemetry;
    public threshold_Pipeline(Telemetry t) {
        telemetry = t;
    }

    public threshold_Pipeline() {

    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingmatrix);
        input.copyTo(Purple);
        input.copyTo(Yellow);
        input.copyTo(Green);

        //color scales
        Imgproc.cvtColor(Purple, Purple, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Green, Green, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV);

        Scalar top = new Scalar(355, 100, 100);
        Scalar bottom = new Scalar(0, 0, 0);
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
        Core.inRange(workingmatrix, bottom, top , workingmatrix);

        Purple.submat(center);
        Green.submat(center);
        Yellow.submat(center);
        workingmatrix.submat(center);

        Scalar greenrect = new Scalar(130/2, 100, 50);
        Scalar yellowrect = new Scalar(204/2, 100, 0);
        Scalar purplerect = new Scalar(274/2, 100, 99);

        color = Core.sumElems(workingmatrix).val[0] / center.area() / 255;
        double PURPLE = Core.sumElems(Purple).val[0] / center.area() / 255;
        double GREEN = Core.sumElems(Green).val[0] / center.area() / 255;
        double YELLOW = Core.sumElems(Yellow).val[0] / center.area() / 255;






//        workingmatrix.release();




        if (color > Threshold){
            telemetry.addData("working", "well");
            telemetry.update();

            if (YELLOW > Threshold){
////            location = Location.yellow;
                Imgproc.rectangle(input, center, yellowrect, 5);
                telemetry.addData("Color", "Yellow");
            }else if (GREEN > Threshold){
////            location = Location.green;
                Imgproc.rectangle(input, center, greenrect, 5);
                telemetry.addData("Color", "Green");
            }else if (PURPLE > Threshold){
////            location = Location.purple;
                Imgproc.rectangle(input, center, purplerect, 6);
                telemetry.addData("Color", "Purple");
            }

        }
        return input;
    }
}
