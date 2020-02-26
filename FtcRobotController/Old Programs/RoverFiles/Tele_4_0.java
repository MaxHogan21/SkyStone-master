package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tele_4_0")
@Disabled

public class Tele_4_0 extends OpMode
{
    //Finds the Hardware map to get all motors, sensors, and configurations
    org.firstinspires.ftc.teamcode.OldPrograms.RoverFiles.ROVERHardware1 ROVER = new org.firstinspires.ftc.teamcode.ROVERHardware1();

    public void init ()
    {
        //The init of the hardware map itself, RunMode, and Direction of motors
        ROVER.init(hardwareMap);

        ROVER.leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ROVER.rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Why do we set 3 motors backwards?
        //Because we use REV Core Hex Motors that are reversible,
        //only one motor is flipped on its x axis, therefore
        //we only set the direction backwards on either one or three of the motors

        ROVER.leftf.setDirection(DcMotor.Direction.REVERSE);
        ROVER.leftb.setDirection(DcMotor.Direction.REVERSE);
        ROVER.rightf.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {

        //Our mechanum code is simple, we take each stick input, create a relative speed

        double drive;   // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotate;  // Power for rotating the robot
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;
        ROVER.leftf.setPower(frontLeftPower);
        ROVER.leftb.setPower(backLeftPower);
        ROVER.rightf.setPower(frontRightPower);
        ROVER.rightb.setPower(backRightPower);
        telemetry.addData("Left Front Power", frontLeftPower);
        telemetry.addData("Left Rear Power", backLeftPower);
        telemetry.addData("Right Front Power", frontRightPower);
        telemetry.addData("Right Rear Power", backRightPower);


        //Both of the DC Motor controls for our mineral collection arm
        ROVER.ext.setPower(-gamepad2.left_stick_y);
        ROVER.lift.setPower(-gamepad2.right_stick_y);

        //This is a driver preference segment, as our primary driver likes to use the
        //dpad to move in more precise movements
        if(gamepad1.dpad_up) {
            ROVER.leftf.setPower(-.8);
            ROVER.leftb.setPower(-.8);
            ROVER.rightf.setPower(-.8);
            ROVER.rightb.setPower(-.8);
        }

        if(gamepad1.dpad_down){
            ROVER.leftf.setPower(.8);
            ROVER.leftb.setPower(.8);
            ROVER.rightf.setPower(.8);
            ROVER.rightb.setPower(.8);
        }

        if(gamepad1.dpad_left){
            ROVER.leftf.setPower(-.8);
            ROVER.leftb.setPower(-.8);
            ROVER.rightf.setPower(.8);
            ROVER.rightb.setPower(.8);

        }

        if(gamepad1.dpad_right){
            ROVER.leftf.setPower(.8);
            ROVER.leftb.setPower(.8);
            ROVER.rightf.setPower(-.8);
            ROVER.rightb.setPower(-.8);
        }

        //Our Lander Climbing Controls
        if(gamepad2.dpad_up){
            ROVER.climb.setPower(1);
        } else if(gamepad2.dpad_down){
            ROVER.climb.setPower(-1);
        } else {
            ROVER.climb.setPower(0);
        }
        //Our mineral collection continuous servo control
        if(gamepad2.a){
            ROVER.sweep.setPower(.5);
        } else if (gamepad2.b){
            ROVER.sweep.setPower(-.5);
        } else if (gamepad2.y) {
            ROVER.sweep.setPower(0);
        }
        //The hook on our quick latch for no slipping when climbing the lander
        if (gamepad1.left_bumper){
            ROVER.grab.setPosition(90);
        }else if(gamepad1.right_bumper){
            ROVER.grab.setPosition(-90);
        }

        if (gamepad2.x){
            ROVER.lift.setPower(10);
            while (ROVER.sleep < ROVER.threshold){
                ROVER.sleep += 1;
            }
            ROVER.lift.setPower(0);
        }
    }
}

