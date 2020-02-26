package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DepotNODEL")
@Disabled

public class DepotNODEL extends LinearOpMode {

    ROVERHardware1 ROVER = new ROVERHardware1();

    public void runOpMode() throws InterruptedException {
        ROVER.init(hardwareMap);

        ROVER.leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ROVER.rightb.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        sleep(1000);

        //
        land();
        moveForward(.5);
        sleep(2800);
        halt();
        sleep(1000);
        ROVER.sweep.setPower(.5);
        sleep(2000);
        ROVER.sweep.setPower(0);
        sleep(500);
        turnRight(.75);
        sleep(800);
        halt();
        sleep(1000);
        moveForward(.5);
        sleep(800);
        halt();
        sleep(500);
        turnRight(.75);
        sleep(600);
        halt();
        moveForward(.5);
        sleep(3000);
        halt();
        moveForward(.5);
        sleep(100);
        halt();

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
        sleep(5000);
        ROVER.climb.setPower(0);
        ROVER.grab.setPosition(-90);
        sleep(1000);
        ROVER.climb.setPower(1);
        sleep(750);
        ROVER.climb.setPower(0);
        sleep(1000);
    }

}
