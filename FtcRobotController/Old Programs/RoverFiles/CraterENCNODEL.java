package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="CraterENCNODEL")
@Disabled
public class CraterENCNODEL extends LinearOpMode {

    ROVERHardware1 ROVER = new ROVERHardware1();


    public void runOpMode() throws InterruptedException {
        ROVER.init(hardwareMap);

        telemetry.addData("Status","Resetting Encoders");
        telemetry.update();
        ROVER.leftf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ROVER.leftb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ROVER.rightf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ROVER.rightb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ROVER.rightf.setDirection(DcMotor.Direction.REVERSE);
        ROVER.rightb.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        sleep(1000);

        telemetry.addData("Path","Complete");
        telemetry.update();

        ROVER.leftf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ROVER.leftb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ROVER.rightf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ROVER.rightb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveForwardENC(20);





    }

    public void moveForward(double value){
        ROVER.leftf.setPower(-value);
        ROVER.leftb.setPower(-value);
        ROVER.rightf.setPower(1.6*-value);
        ROVER.rightb.setPower(1.6*-value);
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
        ROVER.rightf.setPower(1.6*-value);
        ROVER.rightb.setPower(1.6*-value);
    }

    public void turnRight(double value){
        ROVER.leftf.setPower(-value);
        ROVER.leftb.setPower(-value);
        ROVER.rightf.setPower(1.6*value);
        ROVER.rightb.setPower(1.6*value);
    }

    public void moveBackwards(double value){
        ROVER.leftf.setPower(value);
        ROVER.leftb.setPower(value);
        ROVER.rightf.setPower(value);
        ROVER.rightb.setPower(value);
    }

    public void land(){
        ROVER.climb.setPower(1);
        sleep(5500);
        ROVER.climb.setPower(0);
        ROVER.grab.setPosition(-90);
        sleep(1000);
        ROVER.climb.setPower(1);
        sleep(750);
        ROVER.climb.setPower(0);
        sleep(1000);
    }

    public void moveForwardENC(int inches){
        double COUNTS_PER_MOTOR_REV = 288;
        double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /(WHEEL_DIAMETER_INCHES * 3.1415);
        double finalcount = inches*COUNTS_PER_INCH;

        ROVER.leftf.setTargetPosition((int)finalcount);
        ROVER.rightf.setTargetPosition((int)finalcount);
        ROVER.leftb.setTargetPosition((int)finalcount);
        ROVER.rightb.setTargetPosition((int)finalcount);
        ROVER.leftf.setPower(-1);
        ROVER.rightf.setPower(-1);
        ROVER.leftb.setPower(1);
        ROVER.rightb.setPower(-1);
        ROVER.leftf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ROVER.rightf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ROVER.leftb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ROVER.rightb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (ROVER.leftf.isBusy()|| ROVER.rightf.isBusy()|| ROVER.leftb.isBusy()|| ROVER.rightb.isBusy()){
            telemetry.addData("Status", "Running to Objective");
            telemetry.update();
        }

        ROVER.leftf.setPower(0);
        ROVER.rightf.setPower(0);
        ROVER.leftb.setPower(0);
        ROVER.rightb.setPower(0);

    }



}
