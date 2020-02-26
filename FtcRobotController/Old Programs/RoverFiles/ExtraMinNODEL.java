package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="ExtraMinNODEL")
@Disabled

public class ExtraMinNODEL extends LinearOpMode {

    ROVERHardware1 ROVER = new ROVERHardware1();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    private static final String VUFORIA_KEY = "AQMXapn/////AAABmUkIKliu8UhAlsC0hI8AZ/gGzHN693N16RDIV6KvJMcygzolaMYuceUhBHEFuw9JwHBpBSS2OV/BEczUwrgYp9iMPev1ooBl10M89qxmmps38aXL7YycUEe3FTH/0YnvFmPCqUc60Hr0rpAgYqcbmKNfGPF7GCVYsHDGTjUUJAepX5HiX1UUES01Wji5ZArDu9A3oTSMvjSVULFB6wLXRKK8Qk8p/sh3NZsg11NtgjePsUckyvJXTVxTaRwltAWBh9eLZsMwHsZD5pcUSsJwXQFIqGwYE7T7fTMGhPZw/V1bsKTzp7rw5ErPbeBvLUzyHe9DlIyLbJqQ1pIoF9UP+PbQgz3HHf0F7bsKpc3EGa0l";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() throws InterruptedException {
        ROVER.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        ROVER.leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ROVER.rightb.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        sleep(250);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    sleep(250);
                                    land();
                                    moveForward(.5);
                                    sleep(700);
                                    halt();
                                    turnLeft(.5);
                                    sleep(620);
                                    halt();
                                    sleep(500);
                                    moveForward(.5);
                                    sleep(900);
                                    halt();
                                    sleep(500);
                                    moveBackwards(.5);
                                    sleep(725);
                                    halt();
                                    sleep(500);
                                    turnRight(.5);
                                    sleep(620);
                                    halt();
                                    rest();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    sleep(250);
                                    land();
                                    moveForward(.5);
                                    sleep(700);
                                    halt();
                                    turnRight(.5);
                                    sleep(900);
                                    halt();
                                    sleep(500);
                                    moveForward(.5);
                                    sleep(900);
                                    halt();
                                    sleep(500);
                                    moveBackwards(.5);
                                    sleep(750);
                                    halt();
                                    sleep(500);
                                    turnLeft(.5);
                                    sleep(850);
                                    halt();
                                    restRIGHT();
                                } else {
                                    sleep(250);
                                    land();
                                    moveForward(.5);
                                    sleep(700);
                                    halt();
                                    moveForward(.5);
                                    sleep(650);
                                    halt();
                                    sleep(500);
                                    moveBackwards(.5);
                                    sleep(600);
                                    halt();
                                    sleep(500);
                                    rest();
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

    }

    public void moveForward(double value){
        ROVER.leftf.setPower(-value);
        ROVER.leftb.setPower(-value);
        ROVER.rightf.setPower(-value);
        ROVER.rightb.setPower(-value);
    }

    public void halt (){
        ROVER.leftf.setPower(0);
        ROVER.leftb.setPower(0);
        ROVER.rightf.setPower(0);
        ROVER.rightb.setPower(0);
    }

    public void turnLeft(double value){
        ROVER.leftf.setPower(value);
        ROVER.leftb.setPower(value);
        ROVER.rightf.setPower(-value);
        ROVER.rightb.setPower(-value);
    }

    public void turnRight(double value){
        ROVER.leftf.setPower(-value);
        ROVER.leftb.setPower(-value);
        ROVER.rightf.setPower(value);
        ROVER.rightb.setPower(value);
    }

    public void moveBackwards(double value){
        ROVER.leftf.setPower(value);
        ROVER.leftb.setPower(value);
        ROVER.rightf.setPower(value);
        ROVER.rightb.setPower(value);
    }
    public void land(){
        ROVER.climb.setPower(1);
        sleep(4700);
        ROVER.climb.setPower(0);
        ROVER.grab.setPosition(-90);
        sleep(1000);
        ROVER.climb.setPower(1);
        sleep(750);
        ROVER.climb.setPower(0);
        sleep(1000);
    }

    public void restRIGHT(){
        turnLeft(.75);
        sleep(750);
        halt();
        sleep(500);
        moveForward(.5);
        sleep(850);
        halt();
        sleep(250);
        turnLeft(.75);
        sleep(300);
        halt();
        sleep(500);
        moveForward(.5);
        sleep(2000);
        halt();
        sleep(250);
        moveForward(.5);
        while (opModeIsActive() && ROVER.rangeSensor.rawUltrasonic() > 60){
        }
        halt();
        sleep(500);
        ROVER.dump.setPosition(-90);
        sleep(1000);
        ROVER.dump.setPosition(70);
        sleep(250);
        turnLeft(.75);
        sleep(250);
        halt();
        moveBackwards(.75);
        sleep(3300);
        halt();
        sleep(250);
        moveBackwards(.25);
        sleep(400);
        halt();
    }

    public void rest(){
        turnLeft(.75);
        sleep(750);
        halt();
        sleep(500);
        moveForward(.5);
        while (opModeIsActive() && ROVER.rangeSensor.rawUltrasonic() > 45){
            telemetry.addData("raw ultrasonic", ROVER.rangeSensor.rawUltrasonic());
            telemetry.update();
        }
        halt();
        sleep(250);
        turnLeft(.75);
        sleep(325);
        halt();
        sleep(500);
        moveForward(.5);
        sleep(2000);
        halt();
        sleep(250);
        moveForward(.5);
        while (opModeIsActive() && ROVER.rangeSensor.rawUltrasonic() > 60){
        }
        halt();
        sleep(500);
        ROVER.dump.setPosition(-90);
        sleep(1000);
        ROVER.dump.setPosition(70);
        sleep(250);
        turnLeft(.75);
        sleep(150);
        halt();
        moveBackwards(.75);
        sleep(3300);
        halt();
        sleep(250);
        moveBackwards(.25);
        sleep(400);
        halt();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
