package Old_not_useful;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import Worlds.SKYHardware;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "BlueBuildVision")
@Disabled


// vision
public class BlueBuildA3 extends LinearOpMode {


    SKYHardware SKY = new SKYHardware();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public double globalAngle;

    int blockcount = 0;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AQMXapn/////AAABmUkIKliu8UhAlsC0hI8AZ/gGzHN693N16RDIV6KvJMcygzolaMYuceUhBHEFuw9JwHBpBSS2OV/BEczUwrgYp9iMPev1ooBl10M89qxmmps38aXL7YycUEe3FTH/0YnvFmPCqUc60Hr0rpAgYqcbmKNfGPF7GCVYsHDGTjUUJAepX5HiX1UUES01Wji5ZArDu9A3oTSMvjSVULFB6wLXRKK8Qk8p/sh3NZsg11NtgjePsUckyvJXTVxTaRwltAWBh9eLZsMwHsZD5pcUSsJwXQFIqGwYE7T7fTMGhPZw/V1bsKTzp7rw5ErPbeBvLUzyHe9DlIyLbJqQ1pIoF9UP+PbQgz3HHf0F7bsKpc3EGa0l";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);

        BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();

        imuparameters.mode = BNO055IMU.SensorMode.IMU;
        imuparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.loggingEnabled = false;

        //Finds the BNO055IMU class and gets its information
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initializes IMU Parameters
        imu.initialize(imuparameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
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
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        resetAngle();
        // defaults plate servo and kicker to neutral positions
        SKY.grabHolder.setPosition(.5);
        //SKY.smackServo.setPosition(1);
        sleep(500);
        // moves back and turns to line up with build plate
        moveBackward(.5);
        sleep(300);
        halt();
        sleep(200);
        rotateNR(-24,.25);
        sleep(100);
        // moves to build plate
        moveBackwardsEncoders(.5,1150);
        sleep(200);
        // grabs build plate
        SKY.grabHolder.setPosition(1);
        moveBackward(.5);
        sleep(300);
        halt();
        sleep(1200);
        // moves forward with build plate
        moveForward(.75);
        sleep(150);
        halt();
        sleep(200);
        //  turns with build plate
        SKY.left.setPower(1);
        SKY.right.setPower(.13);
        sleep(1500);
        halt();
        // moves forward with build plate again
        moveForwardEncoders(.6,320);
        // rotates with build plate
        rotateNR(35,1);
        sleep(500);
        // moves build plate into wall
        moveBackward(.35);
        sleep(1200);
        halt();
        sleep(200);
        // lets go of build plate
        SKY.grabHolder.setPosition(.5);
        halt();
        sleep(50);
        moveForwardEncoders(.65, 400);
        sleep(50);
        turnRight(1, 200);
        sleep(50);
        moveForwardEncoders(.65, 250);
        sleep(50);
        turnLeft(1, 195);
        sleep(200);
        // moves forward to get stone
        moveForwardEncoders(.45,3210);
        sleep(200);
        // activate vuforia
        targetsSkyStone.activate();
        sleep(500);
        // start loop to find block location
        while (!isStopRequested() && !targetVisible && blockcount < 2) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());
                telemetry.update();
                targetVisible = true;
                sleep(1000);
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
            moveForwardEncoders(.25, 355);
            sleep(1000);
        }
        // shuts off vuforia
        targetsSkyStone.deactivate();
        sleep(50);
        moveArmB2(1,1000);
        sleep(50);
        moveArmB2(-1,900);
        sleep(50);
        moveBackwardsEncoders(.65,3000 + blockcount);
        sleep(50);
        moveArmB2(1,500);
        sleep(50);
        moveForwardEncoders(.65,1200);
        sleep(50);
        halt();

        /*
        rotateNR(65,.4);
        sleep(200);
        moveForwardEncoders(.5,400);
        sleep(1000);
        SKY.grabRight.setPosition(.5);
        sleep(800);
        halt();
        SKY.grabLeft.setPosition(1);
        SKY.grabRight.setPosition(0);
        sleep(800);
        halt();
        moveBackwardsEncoders(.5,280);
        sleep(200);
        turnRight(.5,400);
        rotate(30,.5);
        sleep(200);
        moveBackwardsEncoders(.5,1980);
        sleep(200);
        turnLeft(.5,950);
        sleep(200);
        SKY.armV.setPower(-.75);
        sleep(1500);
        SKY.armV.setPower(0);
        moveForward(.3);
        sleep(500);
        halt();
        SKY.armV.setPower(.75);
        sleep(1500);
        SKY.armV.setPower(0);
        SKY.grabRight.setPosition(1);
        SKY.grabLeft.setPosition(0);
        sleep(1000);
        halt();*/


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
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */

    //We experimentally determined the Z axis is the axis we want to use for heading angle.
    // We have to process the angle because the imu works in euler angles so the Z axis is
    // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
    // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

    private double getAngle()
    {

        Orientation angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    //See if we are moving in a straight line and if not return a power correction value.
    //return power adjustment, + is adjust left - is adjust right.
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction;
        double angle;
        double gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }



    // This rotates left or right a number of degrees. Does not support turning more than 180 degrees.
    //Degrees to turn, + is left - is right
    private void rotate(int degrees, double power)
    {
        double leftPower;
        double rightPower;
        //Restarts IMU for tracking
        resetAngle();
        // getAngle() returns + when turning left and - when turning right
        // clockwise (right).
        if (degrees < 0) {
            // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0){
            // turn left.
            leftPower = power;
            rightPower = -power;
        } else
            return;
        // set power to rotate.
        SKY.left.setPower(leftPower);
        SKY.right.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
        // turn the motors off.
        halt();
        // wait for rotation to stop.
        sleep(1000);
        // reset angle for new heading
        resetAngle();
    }

    private void rotateNR(int degrees, double power)
    {
        double leftPower;
        double rightPower;
        //Restarts IMU for tracking
        // getAngle() returns + when turning left and - when turning right
        // clockwise (right).
        if (degrees < 0) {
            // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0){
            // turn left.
            leftPower = power;
            rightPower = -power;
        } else
            return;
        // set power to rotate.
        SKY.left.setPower(leftPower);
        SKY.right.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
        // turn the motors off.
        halt();
        // wait for rotation to stop.
        sleep(1000);
        // reset angle for new heading
        resetAngle();
    }

    private void rotateBuild(int degrees, double power)
    {
        double leftPower;
        double rightPower;
        //Restarts IMU for tracking
        // getAngle() returns + when turning left and - when turning right
        // clockwise (right).
        if (degrees < 0) {
            // turn right.
            leftPower = -.05*power;
            rightPower = power;
        } else if (degrees > 0){
            // turn left.
            leftPower = power;
            rightPower = -.05*power;
        } else
            return;
        // set power to rotate.
        SKY.left.setPower(leftPower);
        SKY.right.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
                telemetry.update();
            }
        // turn the motors off.
        halt();
        // wait for rotation to stop.
        sleep(1000);
        // reset angle for new heading
        resetAngle();
    }
    public void rampEncoders(int distance){
        SKY.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.left.setTargetPosition(distance);
        SKY.right.setTargetPosition(distance);

        SKY.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SKY.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SKY.left.setPower(.05);
        SKY.right.setPower(.05);

        while( SKY.left.isBusy() && SKY.right.isBusy()){
            while(opModeIsActive()&& SKY.left.getCurrentPosition()< distance) {
                SKY.power = SKY.left.getCurrentPosition() * 1/5000 + 100;
                SKY.left.setPower(SKY.power);
                SKY.right.setPower(SKY.power);
            }
        }


        halt();
        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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