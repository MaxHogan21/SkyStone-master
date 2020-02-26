package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;
import java.util.List;

import Worlds.SKYHardware;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous
//@Disabled
public class Block_Vision_2 extends LinearOpMode {

    SKYHardware SKY = new SKYHardware();
    private int blockcount = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AQMXapn/////AAABmUkIKliu8UhAlsC0hI8AZ/gGzHN693N16RDIV6KvJMcygzolaMYuceUhBHEFuw9JwHBpBSS2OV/BEczUwrgYp9iMPev1ooBl10M89qxmmps38aXL7YycUEe3FTH/0YnvFmPCqUc60Hr0rpAgYqcbmKNfGPF7GCVYsHDGTjUUJAepX5HiX1UUES01Wji5ZArDu9A3oTSMvjSVULFB6wLXRKK8Qk8p/sh3NZsg11NtgjePsUckyvJXTVxTaRwltAWBh9eLZsMwHsZD5pcUSsJwXQFIqGwYE7T7fTMGhPZw/V1bsKTzp7rw5ErPbeBvLUzyHe9DlIyLbJqQ1pIoF9UP+PbQgz3HHf0F7bsKpc3EGa0l";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;


    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    // 1 foot = 375 ticks
    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        // moves arm out initially
        moveArmB2(1, 1150);
        sleep(50);
        // activate vuforia
        targetsSkyStone.activate();
        sleep(400);
        // start loop to find block location
        while (!isStopRequested() && !targetVisible && blockcount < 2) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                telemetry.update();
                targetVisible = true;
                sleep(700);
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
            blockcount += 1;
            // moves forward 8 inches
            moveForwardEncoders(.25, 316);
            sleep(700);
        }
        // shuts off vuforia
        targetsSkyStone.deactivate();
        // move arm out to get skystone
        moveArmB2(1, 2400);
        sleep(50);
        moveArmB1(1, 750);
        sleep(50);
        // move arm in
        moveArmB2(-1, 2250);
        sleep(50);
        moveArmB1(-1, 750);
        //moves out distance determined by block found
        moveForwardEncoders(.7, 2500 - blockcount * 335);
        // drops off block
        moveArmB2(1, 525);
        // moves back into wall
        moveBackwardsEncoders(.7, 3350);
        sleep(200);
        moveBackward(.7);
        sleep(500);
        halt();
        sleep(1000);
        // moves forward to line up with skystone
        moveForwardEncoders(.5, blockcount * 258);
        sleep(50);
        // moves arm out to get skystone
        moveArmB2(1, 2250);
        sleep(50);
        moveArmB1(1, 900);
        sleep(50);
        // moves arm in
        moveArmB2(-1, 2100);
        sleep(50);
        moveArmB1(-1, 900);
        sleep(50);
        // moves forward to drop off skystone
        moveForwardEncoders(.65, 3000 - blockcount * 335);
        // lets go of skystone
        moveArmB2(1, 750);
        // parks
        moveBackwardsEncoders(1,500);
        sleep(50);
        // moves arm back in
        SKY.grabLeft.setPosition(.45);
        SKY.grabRight.setPosition(.55);
        moveArmB2(-1, 700);
        // stops encoders
        SKY.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // stops program
        stop();
    }
    public void moveForwardEncoders(double power, int distance){
        SKY.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.left.setTargetPosition(distance);
        SKY.right.setTargetPosition(distance);

        SKY.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SKY.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveForward(power);

        while( SKY.left.isBusy() && SKY.right.isBusy()){
            telemetry.addLine("Running to position - busy");
            telemetry.update();
        }

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

        while( SKY.left.isBusy() && SKY.right.isBusy()){
            telemetry.addLine("Running to position - busy");
            telemetry.update();
        }

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
        halt();
    }
    public void turnLeft(double power, int time){
        SKY.left.setPower(power);
        SKY.right.setPower(-power);
        sleep(time);
        halt();
    }
    public void moveArmB2(double power, int time){
        SKY.armB2.setPower(power);
        sleep(time);
        SKY.armB2.setPower(0);
    }
    public void moveArmB1(double power, int time){
        SKY.armB1.setPower(power);
        sleep(time);
        SKY.armB1.setPower(0);
    }
}
