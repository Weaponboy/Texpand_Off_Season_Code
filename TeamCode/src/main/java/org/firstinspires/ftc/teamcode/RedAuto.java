package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AutoBot;

import java.util.List;

@Autonomous

public class RedAuto extends LinearOpMode {

    int SlideLevel = 0;
    AutoBot RedBot = new AutoBot();
    Thread myThread = new Thread();

    int counter = 0;
    boolean MarkerDetected = false;
    float MarkerPos = 3;
    int Location = 0;
    int Location2Counter = 0;
    int Location3Counter = 0;
    int For_counter = 0;
    int Counter_if_statment = 0;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };






    private static final String VUFORIA_KEY =
            " AR3EPAL/////AAABmXPb0JQ8qUwHnsHj7nkb+Zs6SEC+RlWYbPl0mJG4o3VrKI0aNhvDLVUj8Lt7xeThyotImJrdlE1s3ft0fKdGrUrxrWpzcAn44zSGjxZA71uE7dUI9Ybku4l0wour9e6LmbenouZ" +
                    "QW+dYioOxWapSzqOIV33yKRaekEdz/BlTTu2UPLR9ELdJMipXH1Eb8fzDnhNIi+masNsV2r/oshdpT1SRYEGVsDfhhgjmjRn3RPnrM15lQH37F29s3xXshpqObbPOCIyYpFzwr+k+eY5jLU2i/qFwtYeZ7SuSPXqnbfZIOQ6mTPtNRSkhysWJKdWKshprQF6BMde8vEpLEjOOBRjiBNkjslK74uCW2xPKdlAW";

    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        RedBot.init(hardwareMap);
        RedBot.ResetEncoders();


        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();







        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            Counter_if_statment++;
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            int i = 0;
            for (Recognition recognition :  updatedRecognitions) {
                i++;
                For_counter++;
                try {
                    Thread.sleep(1);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                // check label to see if the camera now sees a Duck
                if (recognition.getLabel().equals("Duck")) {


                    MarkerPos = recognition.getRight();
                    telemetry.addData("# Object Detected", MarkerPos);

                    if (MarkerPos < 250){

                        Location2Counter++;

                    } else if(MarkerPos > 250){

                        Location3Counter++;
                    }
                } else {
                    MarkerDetected = false;
                }

                telemetry.addData("counter if loop", Counter_if_statment);
                telemetry.addData("Location 2", Location2Counter);
                telemetry.addData("Location 3", Location3Counter);
                telemetry.addData("Counter", counter);
                telemetry.addData("For loop counter", For_counter);
                telemetry.update();

                vuforia.setFrameQueueCapacity(0);


            }

        }
        try {
            Thread.sleep(2000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        if (opModeIsActive()) {
            while (counter<1000) {
                if (updatedRecognitions != null) {
                    Counter_if_statment++;
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition :  updatedRecognitions) {
                        i++;
                        For_counter++;
                        try {
                            Thread.sleep(1);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        // check label to see if the camera now sees a Duck
                        if (recognition.getLabel().equals("Duck")) {


                            MarkerPos = recognition.getRight();
                            telemetry.addData("# Object Detected", MarkerPos);

                            if (MarkerPos < 250){

                                Location2Counter++;

                            } else if(MarkerPos > 250){

                                Location3Counter++;
                            }
                        } else {
                            MarkerDetected = false;
                        }

                        telemetry.addData("counter if loop", Counter_if_statment);
                        telemetry.addData("Location 2", Location2Counter);
                        telemetry.addData("Location 3", Location3Counter);
                        telemetry.addData("Counter", counter);
                        telemetry.addData("For loop counter", For_counter);
                        telemetry.update();

                        vuforia.setFrameQueueCapacity(0);


                    }

                }

                counter++;

            }
            if (Location3Counter < 600 && Location2Counter > 600) {

                Location = 3;
                telemetry.addData("counter if loop", Counter_if_statment);
                telemetry.addData("Counter", counter);
                telemetry.addData("For loop counter", For_counter);
                telemetry.addData("End location:", Location);
                telemetry.addData("Location 2", Location2Counter);
                telemetry.addData("Location 3", Location3Counter);

            }
            if (Location2Counter < 600 && Location3Counter > 600){

                Location = 2;
                telemetry.addData("counter if loop", Counter_if_statment);
                telemetry.addData("Counter", counter);
                telemetry.addData("End location:", Location);
                telemetry.addData("For loop counter", For_counter);
                telemetry.addData("Location 2", Location2Counter);
                telemetry.addData("Location 3", Location3Counter);
            }
            if (Location2Counter > 600 && Location3Counter > 600){

                Location = 1;
                telemetry.addData("counter if loop", Counter_if_statment);
                telemetry.addData("Counter", counter);
                telemetry.addData("End location:", Location);
                telemetry.addData("For loop counter", For_counter);
                telemetry.addData("Location 2", Location2Counter);
                telemetry.addData("Location 3", Location3Counter);
            }
            telemetry.update();

        }

        telemetry.addData("Location:", Location);
        telemetry.update();

        RedBot.DriveDistance(-90,0.75);
        RedBot.LiftSetPossition(3);
        try {
            RedBot.Dumpbucket();
        }catch (InterruptedException e) {
            e.printStackTrace();
        }
        RedBot.LiftSetPossition(0);
        RedBot.StrafeDistance(85,0.75);
        RedBot.DriveDistance(55,0.75);
        RedBot.carousel.setPower(-1);
        RedBot.DriveDistance(10,0.75);
        try {
            Thread.sleep(1800);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        RedBot.carousel.setPower(0);
        RedBot.StrafeDistance(-45,0.75);
        RedBot.TurnDegrees(-90);
        RedBot.DriveDistance(135,0.75);
        RedBot.StrafeDistance(45,0.75);
        RedBot.DriveDistance(90,0.75);

    }


    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.40f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}