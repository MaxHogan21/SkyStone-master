package Old_not_useful;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import Worlds.SKYHardware;

@Autonomous(name = "BlueBuildA")
@Disabled

public class BlueBuildA extends LinearOpMode {


    SKYHardware SKY = new SKYHardware();

    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);

        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        moveBackward(.5);
        sleep(200);
        turnRight(.5,250);
        moveBackwardsEncoders(.5,1120);
        sleep(200);
        SKY.grabHolder.setPosition(1);
        sleep(1000);
        SKY.left.setPower(1);
        SKY.right.setPower(.08);
        sleep(2000);
        halt();
        sleep(1000);
        moveBackward(.5);
        sleep(1200);
        halt();
        sleep(500);
        SKY.grabHolder.setPosition(.5);
        halt();
        moveForwardEncoders(.5,2500);
        sleep(200);
        turnLeft(.5,350);
        halt();
        sleep(200);
        moveForwardEncoders(.5,280);
        sleep(1000);
        SKY.grabRight.setPosition(.5);
        sleep(1000);
        halt();
        SKY.grabLeft.setPosition(1);
        SKY.grabRight.setPosition(0);
        sleep(1000);
        halt();
        moveBackwardsEncoders(.5,280);
        sleep(200);
        turnRight(.5,300);
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
}