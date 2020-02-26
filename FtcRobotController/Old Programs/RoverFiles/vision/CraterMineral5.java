package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.ROVERHardware1;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "CraterMineral5")
@Disabled
public class CraterMineral5 extends LinearOpMode{

    ROVERHardware1 ROVER = new ROVERHardware1();
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() throws InterruptedException {

        ROVER.init(hardwareMap);

        //We chose sensors over encoders for more accurate readings
        ROVER.leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ROVER.rightb.setDirection(DcMotor.Direction.REVERSE);

        //All the required parameters for IMU
        BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();

        imuparameters.mode = BNO055IMU.SensorMode.IMU;
        imuparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.loggingEnabled = false;

        //Finds the BNO055IMU class and gets its information
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initializes IMU Parameters
        imu.initialize(imuparameters);

        //Set Digital Channel
        ROVER.magnentLimit.setMode(DigitalChannel.Mode.INPUT);

        //When init, telemetry updates
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        //This makes sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;//Rear Camera
        parameters.vuforiaLicenseKey = "AQMXapn/////AAABmUkIKliu8UhAlsC0hI8AZ/gGzHN693N16RDIV6KvJMcygzolaMYuceUhBHEFuw9JwHBpBSS2OV/BEczUwrgYp9iMPev1ooBl10M89qxmmps38aXL7YycUEe3FTH/0YnvFmPCqUc60Hr0rpAgYqcbmKNfGPF7GCVYsHDGTjUUJAepX5HiX1UUES01Wji5ZArDu9A3oTSMvjSVULFB6wLXRKK8Qk8p/sh3NZsg11NtgjePsUckyvJXTVxTaRwltAWBh9eLZsMwHsZD5pcUSsJwXQFIqGwYE7T7fTMGhPZw/V1bsKTzp7rw5ErPbeBvLUzyHe9DlIyLbJqQ1pIoF9UP+PbQgz3HHf0F7bsKpc3EGa0l";

        //This next line is important, we take all the data from our Kotlin files and apply them
        //The last inclusion is a major difference between our program and other teams
        //Some teams want to be able to form a conjecture on the mineral position by inferring
        //We instead choose to look at all three to get the most accurate reading
        vision = new MasterVision(parameters, hardwareMap, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();//Starts the camera on the FTC APP
        vision.enable();// This actually starts the tracking of TensorFlow

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addLine("TensorFlow and IMU are both ready");
        telemetry.update();

        //Start of program
        waitForStart();

        //Stops TensorFlow and allows phones processing power to be allocated elsewhere
        vision.disable();

        //Tells the robot what position gold mineral is in based on last known order seen
        goldPosition = vision.getTfLite().getLastKnownSampleOrder();


        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);//giving feedback
            telemetry.update();

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    telemetry.update();
                    sleep(250);
                    land();
                    ROVER.dump.setPosition(.5);
                    resetAngle();
                    sleep(300);
                    moveForward(.8);
                    sleep(300);
                    halt();
                    rotate(15,.75);
                    sleep(250);
                    moveForward(.8);
                    sleep(800);
                    halt();
                    sleep(250);
                    moveBackwards(.8);
                    sleep(500);
                    halt();
                    sleep(250);
                    rest();
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    telemetry.update();
                    sleep(250);
                    land();
                    ROVER.dump.setPosition(.5);
                    resetAngle();
                    sleep(250);
                    moveForward(.8);
                    sleep(1000);
                    halt();
                    sleep(250);
                    moveBackwards(.7);
                    sleep(500);
                    halt();
                    sleep(250);
                    rest();
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    telemetry.update();
                    sleep(250);
                    land();
                    ROVER.dump.setPosition(.5);
                    resetAngle();
                    sleep(300);
                    moveForward(.8);
                    sleep(300);
                    halt();
                    rotate(-20,.75);
                    sleep(250);
                    moveForward(.8);
                    sleep(900);
                    halt();
                    sleep(250);
                    moveBackwards(.8);
                    sleep(600);
                    halt();
                    sleep(250);
                    rest();
                    break;
                case UNKNOWN:
                    //If for whatever reason the algorithm can't pick up the correct mineral
                    //The robot will still run autonomous and run the center mineral
                    telemetry.addLine("Like Scoobs, I am going to use .5 percent of my power");//Driver's Inclusion :)
                    telemetry.update();
                    sleep(250);
                    land();
                    ROVER.dump.setPosition(.5);
                    resetAngle();
                    sleep(250);
                    moveForward(1);
                    sleep(1000);
                    halt();
                    sleep(250);
                    moveBackwards(1);
                    sleep(600);
                    halt();
                    sleep(250);
                    rest();
                    break;
            }

            telemetry.update();
        }

        //TensorFlow shutdown
        vision.shutdown();
    }
    //These are easy methods that quickly allow us to move the robot with one line
    public void moveForward(double value){
        ROVER.leftf.setPower(-value);
        ROVER.leftb.setPower(-value);
        ROVER.rightf.setPower(-value);
        ROVER.rightb.setPower(-value);
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

    public void strafeRight(double value){
        ROVER.leftf.setPower(-value);
        ROVER.leftb.setPower(value);
        ROVER.rightf.setPower(value);
        ROVER.rightb.setPower(-value);
    }

    public void strafeLeft(double value){
        ROVER.leftf.setPower(value);
        ROVER.leftb.setPower(-value);
        ROVER.rightf.setPower(-value);
        ROVER.rightb.setPower(value);
    }

    //This method stops all motion of the drivetrain
    public void halt(){
        ROVER.leftf.setPower(0);
        ROVER.leftb.setPower(0);
        ROVER.rightf.setPower(0);
        ROVER.rightb.setPower(0);
    }
    //Method that makes the arm extend for us to land correctly
    public void land(){
        ROVER.climb.setPower(1);
        while (ROVER.magnentLimit.getState()){
            telemetry.addData("State:", ROVER.magnentLimit.getState());
            telemetry.update();
        }
        ROVER.climb.setPower(0);
        ROVER.grab.setPosition(-90);
        sleep(1000);
        }
    //Instead of completely reworking the entire loop to run the same code for the 3 conditions,
    //we created a method called "rest()" that is always ran after pushing the gold mineral.
    public void rest(){
        //telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
        //telemetry.update();
        rotate(60, .75);
        sleep(250);
        moveForward(1);
        sleep(1000);
        halt();
        sleep(250);
        moveForward(.8);
        //We use a modern robotics range sensor to move the correct distance at the wall
        while (opModeIsActive() && ROVER.rangeSensor.rawUltrasonic() > 45){
            telemetry.addData("raw ultrasonic", ROVER.rangeSensor.rawUltrasonic());
            telemetry.update();
        }
        halt();
        sleep(250);
        turnLeft(1);
        sleep(250);
        halt();
        strafeRight(1);
        sleep(650);
        halt();
        moveForward(.9);
        sleep(750);
        halt();
        sleep(500);
        moveForward(1);
        while (opModeIsActive() && ROVER.rangeSensor.rawUltrasonic() > 60){}
        halt();
        sleep(500);
        ROVER.dump.setPosition(-90);
        sleep(500);
        turnLeft(.75);
        sleep(50);
        halt();
        moveBackwards(1);
        sleep(1000);
        halt();
        sleep(250);
        moveBackwards(.5);
        while (opModeIsActive() && ROVER.rev2M.getDistance(DistanceUnit.INCH ) > 13){ }
        halt();
        moveForward(1);
        sleep(500);
        halt();
        strafeLeft(1);
        sleep(800);
        halt();
        sleep(250);
        turnLeft(1);
        sleep(900);
        halt();
        sleep(200);
        rotate(-70,.95);
        sleep(250);
        moveForward(.75);
        sleep(1500);
        halt();
        ROVER.lift.setPower(-1);
        sleep(400);
        ROVER.lift.setPower(0);
        ROVER.ext.setPower(1);
        sleep(2500);
        ROVER.ext.setPower(0);
        sleep(200);
        ROVER.lift.setPower(1);
        turnLeft(.75);
        sleep(750);
        halt();
        sleep(750);
        ROVER.lift.setPower(0);
        sleep(250);
        ROVER.lift.setPower(-1);
        sleep(500);
        ROVER.lift.setPower(0);
        sleep(10000);

    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

        ROVER.globalAngle = 0;
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

        ROVER.globalAngle += deltaAngle;

        lastAngles = angles;

        return ROVER.globalAngle;
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
        ROVER.leftf.setPower(leftPower);
        ROVER.leftb.setPower(leftPower);
        ROVER.rightf.setPower(rightPower);
        ROVER.rightb.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES));
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

}
