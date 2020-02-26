package Old_not_useful;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import Worlds.SKYHardware;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorBlock_1")
@Disabled
public class TensorBlock_1 extends LinearOpMode {
    SKYHardware SKY = new SKYHardware();
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "ASmWZqr/////AAAAGTT1hSkQHkAdnfLu4EGOvd4Viq8whTdXVtvPRsoUkuVOHtgPaMjgiL3GLlbIQPIGt9JPuLd/gTNLgwOzUFdcad4CleqkXd0sDGEC7oay+uagw2fMsfOHyga/a6NFz+89V/WX2sg/dgGJXCWSArfvb4xMjXx9fY91gFJ9sXsunNujGB4xUKpBUdF2tGMssErR84JpmyPc5t9FTt9haog15732zHWQvG0UV2sgzRF9EP68DQtYqHYxOhejX2l0lvidMyihjbGG6v6br8JE2Ig8hLjdZ7Iq5PfZxcT/QRi4kpauguz6eyKXjtI6/la+2Lz5EHqGz+taC1dvnvDhlA2IeSvdcF6lVo/UpaXCR2nGWGZI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        SKY.init(hardwareMap);

        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
         first. */
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        sleep(1500);
                        for (Recognition recognition : updatedRecognitions) {
                            sleep(1500);
                            if(recognition.getLabel() == LABEL_SECOND_ELEMENT){
                                if(recognition.getLeft() < 350){
                                    moveForwardEncoders(.5, 200);
                                    sleep(1000);
                                    SKY.armB1.setPower(1);
                                    sleep(1700);
                                    SKY.armB1.setPower(0);
                                    sleep(50);
                                    SKY.armB2.setPower(-1);
                                    sleep(2500);
                                    SKY.armB2.setPower(0);
                                    sleep(50);
                                    SKY.armB1.setPower(-1);
                                    sleep(2000);
                                    SKY.armB1.setPower(0);
                                    sleep(50);
                                    SKY.armB2.setPower(1);
                                    sleep(2000);
                                    SKY.armB2.setPower(0);
                                    moveForwardEncoders(.6, 1000);
                                    SKY.armB2.setPower(-1);
                                    sleep(700);
                                    SKY.armB2.setPower(0);
                                    halt();
                                    sleep(1000);
                                    moveBackwardsEncoders(.6,500);
                                } else if (recognition.getLeft() >= 350){
                                    SKY.armB1.setPower(1);
                                    sleep(1700);
                                    SKY.armB1.setPower(0);
                                    sleep(50);
                                    SKY.armB2.setPower(-1);
                                    sleep(2500);
                                    SKY.armB2.setPower(0);
                                    sleep(50);
                                    SKY.armB1.setPower(-1);
                                    sleep(2000);
                                    SKY.armB1.setPower(0);
                                    sleep(50);
                                    SKY.armB2.setPower(1);
                                    sleep(2000);
                                    SKY.armB2.setPower(0);
                                    moveForwardEncoders(.6, 1200);
                                    SKY.armB2.setPower(-1);
                                    sleep(700);
                                    SKY.armB2.setPower(0);
                                    halt();
                                    sleep(1000);
                                    moveBackwardsEncoders(.6,500);
                                }
                            } else {
                                moveBackwardsEncoders(.5, 200);
                                sleep(1000);
                                SKY.armB1.setPower(1);
                                sleep(1700);
                                SKY.armB1.setPower(0);
                                sleep(50);
                                SKY.armB2.setPower(-1);
                                sleep(2500);
                                SKY.armB2.setPower(0);
                                sleep(50);
                                SKY.armB1.setPower(-1);
                                sleep(2000);
                                SKY.armB1.setPower(0);
                                sleep(50);
                                SKY.armB2.setPower(1);
                                sleep(2000);
                                SKY.armB2.setPower(0);
                                moveForwardEncoders(.6, 1400);
                                SKY.armB2.setPower(-1);
                                sleep(700);
                                SKY.armB2.setPower(0);
                                halt();
                                sleep(1000);
                                moveBackwardsEncoders(.6,500);
                            }
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
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

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void moveForwardEncoders(double power, int distance){
        SKY.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.left.setTargetPosition(distance);
        SKY.right.setTargetPosition(distance);

        SKY.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SKY.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveForward(power);

        while( SKY.left.isBusy() && SKY.right.isBusy()){}

        halt();
        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveBackwardsEncoders(double power, int distance){
        SKY.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.left.setTargetPosition(-distance);
        SKY.right.setTargetPosition(-distance);

        SKY.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SKY.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveBackward(power);

        while( SKY.left.isBusy() && SKY.right.isBusy()){}

        halt();
        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward (double power){
        SKY.left.setPower(power);
        SKY.right.setPower(power);
    }

    public void moveBackward (double power){
        SKY.left.setPower(-power);
        SKY.right.setPower(-power);

    }

    public void halt(){
        SKY.left.setPower(0);
        SKY.right.setPower(0);
    }

    public void turnRight(double power, int time){
        SKY.left.setPower(-power);
        SKY.right.setPower(power);
        sleep(time);
        SKY.left.setPower(0);
        SKY.right.setPower(0);
    }
    public void turnLeft(double power, int time){
        SKY.left.setPower(power);
        SKY.right.setPower(-power);
        sleep(time);
        SKY.left.setPower(0);
        SKY.right.setPower(0);
    }
}
