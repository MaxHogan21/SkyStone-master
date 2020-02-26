package Old_not_useful;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import Worlds.SKYHardware;

@Autonomous(name = "BlockA")
@Disabled
public class BlockA extends LinearOpMode {

    SKYHardware SKY = new SKYHardware();

    public void runOpMode() throws InterruptedException{
        SKY.init(hardwareMap);

        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        moveBackward(.3);
        sleep(2200);
        halt();
        sleep(50);
        SKY.armB2.setPower(-1);
        sleep(2500);
        SKY.armB2.setPower(0);
        sleep(50);
        SKY.armB1.setPower(1);
        sleep(825);
        SKY.armB1.setPower(0);
        sleep(50);
        SKY.armB1.setPower(-1);
        sleep(825);
        SKY.armB1.setPower(0);
        sleep(50);
        SKY.armB2.setPower(1);
        sleep(2000);
        SKY.armB2.setPower(0);
        moveForwardEncoders(.8, 3360);
        SKY.armB2.setPower(-1);
        sleep(700);
        SKY.armB2.setPower(0);
        halt();
        sleep(1000);
        moveBackwardsEncoders(.8,600);
        //moveBackwardsEncoders(.8,500);
        /*sleep(1000);
        SKY.armB2.setPower(1);
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
        SKY.armB2.setPower(0); */


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
