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
    Mat Pink = new Mat();
    Mat Yellow = new Mat();
    Mat Blue = new Mat();

    // region of interest
    static final Rect center = new Rect(new Point(100, 100), new
            Point(550, 350));

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
        input.copyTo(Pink);
        input.copyTo(Yellow);
        input.copyTo(Blue);

        //color scales
        Imgproc.cvtColor(Pink, Pink, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Yellow, Yellow, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(Blue, Blue, Imgproc.COLOR_RGB2HSV);

        Scalar pinkup = new Scalar(319/2, 100, 100);
        Scalar pinkdown = new Scalar(292/2, 42, 64);

        Scalar blueup = new Scalar(190/2, 100, 100);
        Scalar bluedown = new Scalar(160/2, 45, 60);

        Scalar yellowup = new Scalar(68/2, 100, 100);
        Scalar yellowdown = new Scalar(50/2, 49, 77);

        // check for colors on the matrix's
        Core.inRange(Pink, pinkdown, pinkup , Pink);
        Core.inRange(Blue, bluedown, blueup, Blue);
        Core.inRange(Yellow, yellowdown, yellowup, Yellow);

        Pink.submat(center);
        Blue.submat(center);
        Yellow.submat(center);

        double PINK = Math.round(Core.mean(Pink).val[2]);
        double BLUE = Math.round(Core.mean(Blue).val[2]);
        double YELLOW = Math.round(Core.mean(Yellow).val[2]);

        Scalar pinkrect = new Scalar(306/2, 100, 100);
        Scalar yellowrect = new Scalar(58/2, 100, 100);
        Scalar bluerect = new Scalar(190/2, 100, 100);

        telemetry.addData("Pink raw value", (int) Core.mean(Pink).val[2]);
        telemetry.addData("Yellow raw value", (int) Core.mean(Yellow).val[2]);
        telemetry.addData("Blue raw value", (int) Core.mean(Blue).val[2]);


        if (YELLOW > COLOR_THRESHOLD){
////            location = Location.yellow;
            Imgproc.rectangle(workingMatrix, center, yellowrect, 5);
            telemetry.addData("Color", "Yellow");
        }else if (PINK > COLOR_THRESHOLD){
////            location = Location.green;
            Imgproc.rectangle(workingMatrix, center, pinkrect, 5);
            telemetry.addData("Color", "Pink");
        }else if (BLUE > COLOR_THRESHOLD){
////            location = Location.purple;
            Imgproc.rectangle(workingMatrix, center, bluerect, 5);
            telemetry.addData("Color", "Blue");
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


