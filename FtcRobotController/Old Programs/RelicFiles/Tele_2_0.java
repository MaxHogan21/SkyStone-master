package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/*
    This is the 2th iteration of the set of TeleOp programs designed by 10101 Binary Bullets
    This program is the primary program used during the driver controlled portion of the game and
    for some testing purposes during prototyping. This program in particular uses the NATARI robot
    and HardwareNatari.java robot class for hardware mapping.
 */

//@TeleOp(name="Tele_2_0")
//@Disabled
public class Tele_2_0 extends OpMode {

    //Inherits hardware from Hardware class
    HardwareNatari NATARI = new org.firstinspires.ftc.teamcode.HardwareNatari();

    public void init() {
        //Initialises the robot with correct hardware
        NATARI.init(hardwareMap);
        //sets the right side in REVERSE
        NATARI.rightf.setDirection(DcMotor.Direction.REVERSE);
        NATARI.rightb.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
        /*
        Our mecanum drive code has gone through several iterations, the following code sets the
        joystick power to a range and then sets the motors to that range power. This worked
        significantly better in comparison to a trigonometric approach.
         */
        float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float RFspeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float RBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;
        //Writes range to value
        LFspeed = Range.clip(LFspeed, -5, 5);
        LBspeed = Range.clip(LBspeed, -5, 5);
        RFspeed = Range.clip(RFspeed, -5, 5);
        RBspeed = Range.clip(RBspeed, -5, 5);
        //Writes Value to Motors
        NATARI.leftf.setPower(-LFspeed);
        NATARI.leftb.setPower(-LBspeed);
        NATARI.rightf.setPower(-RFspeed);
        NATARI.rightb.setPower(-RBspeed);
        //Used Primarily for testing, the telemetry shows the exact speed of each motor
        telemetry.addData("Left_FrontPower", LFspeed);
        telemetry.addData("Left_BackPower", LBspeed);
        telemetry.addData("Right_FrontPower", RFspeed);
        telemetry.addData("Right_BackPower", RBspeed);
        telemetry.update();

        //Function to move the robot perfectly forward without Tank Drive
        if (gamepad1.dpad_up) {
            NATARI.leftf.setPower(.5);
            NATARI.leftb.setPower(.5);
            NATARI.rightf.setPower(.5);
            NATARI.rightb.setPower(.5);
        }
        if(gamepad1.dpad_right){
            NATARI.leftf.setPower(1);
            NATARI.leftb.setPower(0);
            NATARI.rightf.setPower(0);
            NATARI.rightb.setPower(1);
        }
        if(gamepad1.dpad_left){
            NATARI.leftf.setPower(0);
            NATARI.leftb.setPower(1);
            NATARI.rightf.setPower(1);
            NATARI.rightb.setPower(0);
        }

        //Rack System Function
        NATARI.climb.setPower(gamepad2.left_stick_y);
        NATARI.rack1.setPower(gamepad2.right_stick_y);
        /*
        //Vertical Climb Function
        if (gamepad2.dpad_up) {
            NATARI.rack2.setPower(1);
        } else if (gamepad2.dpad_down) {
            NATARI.rack2.setPower(-1);
        } else {
            NATARI.rack2.setPower(0);
        }
        */

        /*
        The following are a set of functions that control a variety of postions on for the 4
        grabbers for the Glyphs. Some are used for positioning, others for our dual purpose
        to grab the Relic.
         */

        if (gamepad2.left_bumper) {
            NATARI.topright.setPosition(.42);
            NATARI.clawright.setPosition(.58);
            NATARI.topleft.setPosition(.55);
            NATARI.clawleft.setPosition(.42);
        }
        if (gamepad2.right_bumper) {
            NATARI.topright.setPosition(0.8);
            NATARI.clawright.setPosition(0.0);
            NATARI.topleft.setPosition(0.2);
            NATARI.clawleft.setPosition(1.0);
        }
        if (gamepad2.y) {
            NATARI.topright.setPosition(.5);
            NATARI.clawright.setPosition(.5);
            NATARI.topleft.setPosition(.5);
            NATARI.clawleft.setPosition(.5);
        }
        if (gamepad2.x) {
            NATARI.topright.setPosition(0.0);
            NATARI.clawright.setPosition(1.0);
            NATARI.topleft.setPosition(1.0);
            NATARI.clawleft.setPosition(0.0);
        }
        if (gamepad2.b) {
            NATARI.topright.setPosition(1.0);
            NATARI.clawright.setPosition(1.0);
            NATARI.topleft.setPosition(0.0);
            NATARI.clawleft.setPosition(0.0);
        }
        if (gamepad2.a) {
            NATARI.topright.setPosition(1.0);
            NATARI.clawright.setPosition(.5);
            NATARI.topleft.setPosition(0.0);
            NATARI.clawleft.setPosition(.5);
        }
        //Relic Specific Grabbers

        /*
        If a problem occured in the autonomous code, the following would reset the jewel arm
        back to its original spot
         */
        if (gamepad1.y) {
            NATARI.jewelarm.setPosition(0.0);
        }
    }
}