package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class AutoBot extends Robot{



    static double ticksperdegree = 12.54;
    static double circumference = 25.1327;

    boolean MarkerDetected = false;
    float MarkerPos = 3;
    int Location = 0;
    int counter = 0;
    int Location1Counter = 0;
    boolean Location1 = Boolean.parseBoolean(null);
    int Location2Counter = 0;
    boolean Location2 = Boolean.parseBoolean(null);;
    int Location3Counter = 0;
    boolean Location3 = Boolean.parseBoolean(null);;

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




    public AutoBot() { }

    public void TurnDegrees(int degrees){
        int ticks = (int) (ticksperdegree*degrees);
        ResetEncoders();

        RF.setTargetPosition(-ticks);
        LF.setTargetPosition(ticks);
        RB.setTargetPosition(-ticks);
        LB.setTargetPosition(ticks);

        RF.setPower(0.75);
        LF.setPower(0.75);
        RB.setPower(0.75);
        LB.setPower(0.75);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.isBusy()){
            try {
                Thread.sleep(50);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void DriveDistance(double distance_cm,double power){
        double wheel_turns_needed = distance_cm/circumference;
        int encoder_driving_target = (int)(wheel_turns_needed*565);
        ResetEncoders();

        RF.setTargetPosition(encoder_driving_target);
        LF.setTargetPosition(encoder_driving_target);
        RB.setTargetPosition(encoder_driving_target);
        LB.setTargetPosition(encoder_driving_target);

        RF.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        LB.setPower(power);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.isBusy()){
            try {
                Thread.sleep(50);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void StrafeDistance(double Strafe_cm,double power){
        double wheel_turns_needed = Strafe_cm/circumference;
        int encoder_driving_target = (int)(1.2*wheel_turns_needed*565);
        ResetEncoders();

        RF.setTargetPosition(-encoder_driving_target);
        LF.setTargetPosition(encoder_driving_target);
        RB.setTargetPosition(encoder_driving_target);
        LB.setTargetPosition(-encoder_driving_target);

        RF.setPower(power);
        LF.setPower(power);
        RB.setPower(power);
        LB.setPower(power);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.isBusy()){
            try {
                Thread.sleep(50);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);



    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.40f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public int DetectMarker(){

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1, 16.0/9.0);
        }
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition :  updatedRecognitions) {
                i++;

                try {
                    Thread.sleep(1);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                // check label to see if the camera now sees a Duck
                if (recognition.getLabel().equals("Duck")) {


                    MarkerPos = recognition.getRight();


                    if (MarkerPos < 250){

                        Location2Counter++;

                    } else if(MarkerPos > 250){

                        Location3Counter++;
                    }
                } else {
                    MarkerDetected = false;
                }


                vuforia.setFrameQueueCapacity(0);


            }

        }
        try {
            Thread.sleep(1000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        while (counter<1000) {
            if (updatedRecognitions != null) {

                int i = 0;
                for (Recognition recognition :  updatedRecognitions) {
                    i++;
                    try {
                        Thread.sleep(1);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    // check label to see if the camera now sees a Duck
                    if (recognition.getLabel().equals("Duck")) {


                        MarkerPos = recognition.getRight();


                        if (MarkerPos < 250){

                            Location2Counter++;

                        } else if(MarkerPos > 250){

                            Location3Counter++;
                        }
                    } else {
                        MarkerDetected = false;
                    }


                    vuforia.setFrameQueueCapacity(0);


                }

            }

            counter++;

        }

        if (Location3Counter > 600) {

            Location = 3;
        }
        else if (Location2Counter > 600) {

            Location = 2;
        }else{

            Location = 1;
        }


        return Location;

    }



}
