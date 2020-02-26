package Worlds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import Worlds.SKYHardware;

@Autonomous
//@Disabled

public class Park extends LinearOpMode {

    SKYHardware SKY = new SKYHardware();

    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);


        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        SKY.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        sleep(1000);
        SKY.grabLeft.setPosition(.5);
        SKY.grabRight.setPosition(.5);
        moveForwardEncoders(1,375);
        // stops program
        SKY.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void moveForward (double power){
        SKY.left.setPower(power);
        SKY.right.setPower(power);
    }
    public void halt(){
        SKY.left.setPower(0);
        SKY.right.setPower(0);
    }
}