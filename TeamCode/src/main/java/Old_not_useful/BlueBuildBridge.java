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

@Autonomous(name = "BlueBuildBridge")
@Disabled

public class BlueBuildBridge extends LinearOpMode {


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
        //SKY.smackServo.setPosition(1);
        sleep(500);
        moveBackward(.5);
        sleep(300);
        halt();
        sleep(200);
        rotateNR(-24,.25);
        sleep(100);
        moveBackwardsEncoders(.5,1150);
        sleep(200);
        SKY.grabHolder.setPosition(1);
        moveBackward(.5);
        sleep(300);
        halt();
        sleep(1200);
        moveForward(.75);
        sleep(150);
        halt();
        sleep(200);
        SKY.left.setPower(1);
        SKY.right.setPower(.13);
        sleep(1500);
        halt();
        moveForwardEncoders(.6,320);
        rotateNR(35,1);
        //rotateBuild(45,1);

        //SKY.left.setPower(1);
        //SKY.right.setPower(.08);
        //sleep(1200);
        //halt();
        //moveForwardEncoders(.8,500);
        //rotateNR(45,1);
        halt();
        sleep(500);
        moveBackward(.5);
        sleep(1200);
        halt();
        sleep(200);
        SKY.grabHolder.setPosition(.5);
        halt();
        turnLeft(.5,350);
        sleep(13000);
        moveForward(.5);
        sleep(1500);
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
}

