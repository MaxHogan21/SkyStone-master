package Old_not_useful;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import Worlds.SKYHardware;

@Autonomous(name = "BlueBuildA2", group = "Functional")
@Disabled

public class BlueBuildA2 extends LinearOpMode {


    SKYHardware SKY = new SKYHardware();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public double globalAngle;

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
        sleep(200);
        // moves forward to get stone
        moveForwardEncoders(.45,3210);
        sleep(200);
        // moves arm forward to get stone
        moveArmB2(1,2000);
        sleep(75);
        // takes in stone
        moveArmB2(-1,1500);
        //moves back to build plate
        moveBackwardsEncoders(.75,3300);
        sleep(50);
        // kicks stone onto build plate
        //SKY.smackServo.setPosition(.3);
        sleep(500);
        // lets go of stone
        moveArmB2(1,500);
        halt();
        sleep(1000);
        // parks (and ramps up)
        moveForwardEncoders(.5, 560);
        sleep(200);
        moveForwardEncoders(.8,690);
        // moves arm back in
        moveArmB2(-1,500);
        halt();



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
