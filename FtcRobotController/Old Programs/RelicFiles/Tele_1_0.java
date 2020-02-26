package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * Created by FIRSTMentor on 9/20/2017.
 */

//@TeleOp(name="Tele_1_0")
//@Disabled

public class Tele_1_0 extends OpMode
{
    DcMotor leftf;
    DcMotor leftb;
    DcMotor rightf;
    DcMotor rightb;
    Servo bottomright;
    Servo bottomleft;
    Servo topright;
    Servo topleft;
    DcMotor rack1;
    //DcMotor rack2;
    DcMotor climb;
    //Servo jewelarm;
    Servo extarm;


    public void init ()
    {
        leftf = hardwareMap.dcMotor.get("left_front_motor");
        leftb = hardwareMap.dcMotor.get("left_back_motor");
        rightf = hardwareMap.dcMotor.get("right_front_motor");
        rightb = hardwareMap.dcMotor.get("right_back_motor");

        topleft = hardwareMap.servo.get("top_Left_motor");
        topright = hardwareMap.servo.get("top_Right_motor");
        bottomright = hardwareMap.servo.get("claw_Right_motor");
        bottomleft = hardwareMap.servo.get("claw_Left_motor");
        climb = hardwareMap.dcMotor.get("vertical_climb");
        rack1 = hardwareMap.dcMotor.get("rack_1");
        //rack2 = hardwareMap.dcMotor.get("rack_2");
        //jewelarm = hardwareMap.servo.get("jewel_motor");
        extarm = hardwareMap.servo.get("ext_arm");

        //jewelarm = hardwareMap.servo.get("jewel_motor");

        rightf.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
    }


    public void loop() {

        float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float RFspeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float RBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;
        //writes range to value
        LFspeed = Range.clip (LFspeed, -5, 5);
        LBspeed = Range.clip (LBspeed, -5, 5);
        RFspeed = Range.clip (RFspeed, -5, 5);
        RBspeed = Range.clip (RBspeed, -5, 5);
        // Writes Value to Motors
        leftf.setPower(-LFspeed);
        leftb.setPower(-LBspeed);
        rightf.setPower(-RFspeed);
        rightb.setPower(-RBspeed);

        //Function to move the robot perfectly foreward without Tank Drive
        if(gamepad1.dpad_up) {
            leftf.setPower(.5);
            leftb.setPower(.5);
            rightf.setPower(.5);
            rightb.setPower(.5);
        }

        if(gamepad1.dpad_down){
            leftf.setPower(-.5);
            leftb.setPower(-.5);
            rightf.setPower(-.5);
            rightb.setPower(-.5);
        }

        //Rack system Function
        climb.setPower(gamepad2.left_stick_y);
        rack1.setPower(gamepad2.right_stick_y);
        /*
        //Vertical Climb Function
        if(gamepad2.dpad_up){
            rack2.setPower(1);
        } else if (gamepad2.dpad_down){
            rack2.setPower(-1);
        } else {
            rack2.setPower(0);
        }
        */

        //Glyph Gripper Function
        if (gamepad2.left_bumper) {
            topright.setPosition(.42);
            bottomright.setPosition(.58);
            topleft.setPosition(.55);
            bottomleft.setPosition(.42);

        }
        if (gamepad2.right_bumper) {
            topright.setPosition(0.8);
            bottomright.setPosition(0.0);
            topleft.setPosition(0.2);
            bottomleft.setPosition(1.0);
        }
        if (gamepad2.y){
            topright.setPosition(.5);
            bottomright.setPosition(.5);
            topleft.setPosition(.5);
            bottomleft.setPosition(.5);
        }
        if (gamepad2.x){
            topright.setPosition(0.0);
            bottomright.setPosition(1.0);
            topleft.setPosition(1.0);
            bottomleft.setPosition(0.0);
        }
        if (gamepad2.b){
            topright.setPosition(1.0);
            bottomright.setPosition(1.0);
            topleft.setPosition(0.0);
            bottomleft.setPosition(0.0);
        }
        if (gamepad2.a){
            topright.setPosition(0.0);
            bottomright.setPosition(0.0);
            topleft.setPosition(1.0);
            bottomleft.setPosition(1.0);
        }

        /*
        //Function for a catch in autonomus
        if (gamepad1.y) {
            jewelarm.setPosition(0.0);
        }
        */


        /*

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        leftf.setPower(v1);
        rightf.setPower(v2);
        leftb.setPower(v3);
        rightb.setPower(v4);


        Mechanum Drive 1.0
        float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float LBspeed = gamepad1.left_stick_y + gamepa d1.left_stick_x;
        float RFspeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float RBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;
        //writes range to value
        LFspeed = Range.clip (LFspeed, -5, 5);
        LBspeed = Range.clip (LBspeed, -5, 5);
        RFspeed = Range.clip (RFspeed, -5, 5);
        RBspeed = Range.clip (RBspeed, -5, 5);
        // Writes Value to Motors
        leftf.setPower(LFspeed);
        leftb.setPower(LBspeed);
        rightf.setPower(RFspeed);
        rightb.setPower(RBspeed);

        */

        //Mechanum Drive 2.0

        /*
        Optional Arcade Drive Version of Mecanum drive

        float LFspeed = gamepad2.left_stick_y + gamepad2.right_stick_x + gamepad2.left_stick_x;
        float LBspeed = gamepad2.left_stick_y + gamepad2.right_stick_x - gamepad2.left_stick_x;
        float RFspeed = gamepad2.left_stick_y - gamepad2.right_stick_x + gamepad2.left_stick_x;
        float RBspeed = gamepad2.left_stick_y - gamepad2.right_stick_x - gamepad2.left_stick_x;

        leftf.setPower(LFspeed);
        leftb.setPower(LBspeed);
        rightf.setPower(RFspeed);
        rightb.setPower(RBspeed);
        */
    }
}
