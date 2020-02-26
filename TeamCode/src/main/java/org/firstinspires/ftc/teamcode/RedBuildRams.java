package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Worlds.SKYHardware;

@Autonomous
//@Disabled

public class RedBuildRams extends LinearOpMode {

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
        //SKY.smackServo.setPosition(0);
        sleep(500);
        // moves back and turns to line up with build plate
        moveBackward(.5);
        sleep(300);
        halt();
        sleep(200);
        rotateNR(24,.25);
        sleep(100);
        // moves to build plate
        moveBackwardsEncoders(.5,1150);
        sleep(200);
        // grabs build plate
        SKY.grabHolder.setPosition(1);
        sleep(300);
        // moves forward with build plate
        moveBackward(.5);
        sleep(300);
        halt();
        sleep(200);
        // moves forward with build plate
        moveForward(.75);
        sleep(75);
        halt();
        sleep(200);
        // turns with build plate
        SKY.left.setPower(.13);
        SKY.right.setPower(1);
        sleep(1500);
        halt();
        // moves forward with build plate again
        moveForwardEncoders(.6,75);
        // rotates with build plate
        rotate_2(-37);
        sleep(500);
        // moves grab servos to hopefully correctly orient skyblock
        SKY.grabRight.setPosition(1);
        SKY.grabLeft.setPosition(.5);
        sleep(100);
        // moves build plate into wall
        moveBackward(.35);
        sleep(1500);
        halt();
        sleep(50);
        // lets go of build plate
        SKY.grabHolder.setPosition(.5);
        sleep(200);
        // moves forward to grab skyblock
        moveForward(.25);
        sleep(1300);
        // grabs skyblock
        SKY.grabRight.setPosition(0);
        SKY.grabLeft.setPosition(1);
        halt();
        sleep(200);
        // turns 180 degrees (gyro usually overshoots a bit) to line up with build plate
        rotate(-165, .5);
        sleep(1000);
        // lifts main arm up
        moveArmV(1,700);
        // moves forward toward build plate
        moveForward(.25);
        sleep(900);
        halt();
        // lowers arm to place block
        moveArmV(-1,900);
        // lets go of block
        SKY.grabRight.setPosition(1);
        SKY.grabLeft.setPosition(0);
        sleep(50);
        // lifts arm up slightly
        moveArmV(1,600);
        // moves away from build plate
        moveBackward(.25);
        sleep(600);
        halt();
        sleep(100);
        // turns toward skybridge so that the back arm is oriented to park over line
        rotate(-20,1);
        sleep(200);
        // moves towards skybridge
        moveForwardEncoders(.7,600);
        sleep(50);
        // moves arm out to park
        moveArmB2(1,1600);
        sleep(50);
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
    public void moveArmB2(double power, int time){
        SKY.armB2.setPower(power);
        sleep(time);
        SKY.armB2.setPower(0);
    }
    public void moveArmV(double power, int time){
        SKY.armV.setPower(power);
        sleep(time);
        SKY.armV.setPower(0);
    }
    private void rotate_2 (int degrees)
    {
        double leftPower;
        double rightPower;
        //Restarts IMU for tracking
        // getAngle() returns + when turning left and - when turning right
        // clockwise (right).
        if (degrees < 0) {
            // turn right.
            leftPower = .13;
            rightPower = 1;
        } else if (degrees > 0){
            // turn left.
            leftPower = 1;
            rightPower = .13;
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
}