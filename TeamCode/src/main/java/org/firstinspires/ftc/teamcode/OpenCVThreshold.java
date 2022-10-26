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
    Mat Red = new Mat();
    Mat Yellow = new Mat();
    Mat Blue = new Mat();
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
        input.copyTo(Red);
        input.copyTo(Yellow);
        input.copyTo(Blue);

        //color scales
        Imgproc.cvtColor(Red, Red, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(Blue, Blue, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2BGR);

        Scalar top = new Scalar(355, 100, 100);
        Scalar bottom = new Scalar(0, 0, 0);
        Scalar redup = new Scalar(130, 150, 255);
        Scalar reddown = new Scalar(100, 42, 0);

        Scalar blueup = new Scalar(190, 100, 100);
        Scalar bluedown = new Scalar(160, 45, 60);

        Scalar yellowup = new Scalar(68, 100, 100);
        Scalar yellowdown = new Scalar(50, 49, 77);

        // check for colors on the matrix's
        Core.inRange(Red, reddown, redup , Red);
        Core.inRange(Blue, bluedown, blueup, Blue);
        Core.inRange(Yellow, yellowdown, yellowup, Yellow);
        Core.inRange(workingmatrix, bottom, top , workingmatrix);

        Red.submat(center);
        Blue.submat(center);
        Yellow.submat(center);
        workingmatrix.submat(center);

        Scalar greenrect = new Scalar(130, 100, 50);
        Scalar yellowrect = new Scalar(204, 100, 0);
        Scalar purplerect = new Scalar(274, 100, 99);

        color = Core.sumElems(workingmatrix).val[0] / center.area() / 255;
        double RED = Core.sumElems(Red).val[2] / center.area() / 255;
        double BLUE = Core.sumElems(Blue).val[0] / center.area() / 255;
        double YELLOW = Core.sumElems(Yellow).val[1] / center.area() / 255;






//        workingmatrix.release();




        if (color > Threshold){
            telemetry.addData("Working", "Well");
            telemetry.update();

            if (YELLOW > Threshold){
////            location = Location.yellow;
                Imgproc.rectangle(input, center, yellowrect, 5);
                telemetry.addData("Color", "YELLOW");
            }else if (BLUE > Threshold){
////            location = Location.green;
                Imgproc.rectangle(input, center, greenrect, 5);
                telemetry.addData("Color", "BLUE");
            }else if (RED > Threshold){
////            location = Location.purple;
                Imgproc.rectangle(input, center, purplerect, 6);
                telemetry.addData("Color", "RED");
            }

        }
        return input;
    }
}
