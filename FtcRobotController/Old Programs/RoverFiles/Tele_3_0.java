package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tele_3_0")
@Disabled

public class Tele_3_0 extends OpMode
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
        //Then clip that speed in a range of -1 to 1,
        // to make sure it doesn't stall motors by going over 1
        float RFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float RBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float LFspeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float LBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;
        //writes range to value
        LFspeed = Range.clip (LFspeed, -1, 1);
        LBspeed = Range.clip (LBspeed, -1, 1);
        RFspeed = Range.clip (RFspeed, -1, 1);
        RBspeed = Range.clip (RBspeed, -1, 1);
        // Writes Value to Motors
        ROVER.leftf.setPower(LFspeed);
        ROVER.leftb.setPower(LBspeed);
        ROVER.rightf.setPower(RFspeed);
        ROVER.rightb.setPower(RBspeed);


        //Both of the DC Motor controls for our mineral collection arm
        ROVER.ext.setPower(-gamepad2.left_stick_y);
        ROVER.lift.setPower(-gamepad2.right_stick_y);

        //This is a driver preference segment, as our primary driver likes to use the
        //dpad to move in more precise movements
        if(gamepad1.dpad_up) {
            ROVER.leftf.setPower(-1);
            ROVER.leftb.setPower(-1);
            ROVER.rightf.setPower(-1);
            ROVER.rightb.setPower(-1);
        }

        if(gamepad1.dpad_down){
            ROVER.leftf.setPower(1);
            ROVER.leftb.setPower(1);
            ROVER.rightf.setPower(1);
            ROVER.rightb.setPower(1);
        }

        if(gamepad1.dpad_left){
            ROVER.leftf.setPower(1);
            ROVER.leftb.setPower(-1);
            ROVER.rightf.setPower(-1);
            ROVER.rightb.setPower(1);

        }

        if(gamepad1.dpad_right){
            ROVER.leftf.setPower(-1);
            ROVER.leftb.setPower(1);
            ROVER.rightf.setPower(1);
            ROVER.rightb.setPower(-1);
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
        //Incase our team marker dump ever gets stuck, we can move it
        if (gamepad2.left_bumper){
            ROVER.dump.setPosition(20);
        }else if(gamepad2.right_bumper){
            ROVER.dump.setPosition(-90);
        }

        /*
        ROVER.leftf.setPower(gamepad1.right_trigger);
        ROVER.rightf.setPower(gamepad1.right_trigger);
        ROVER.leftb.setPower(gamepad1.right_trigger);
        ROVER.rightb.setPower(gamepad1.right_trigger);

        ROVER.leftf.setPower(-gamepad1.left_trigger);
        ROVER.rightf.setPower(gamepad1.left_trigger);
        ROVER.leftb.setPower(gamepad1.left_trigger);
        ROVER.rightb.setPower(gamepad1.left_trigger);
        */


    }
}
