package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Worlds.SKYHardware;


@TeleOp
//@Disabled

public class Tele_2 extends OpMode {

    SKYHardware SKY = new SKYHardware();

    public void init() {
        SKY.init(hardwareMap);
    }

    public void loop() {

        SKY.left.setPower(-gamepad1.left_stick_y);
        SKY.right.setPower(-gamepad1.left_stick_y);

        //turning
        SKY.left.setPower(-gamepad1.right_stick_x);
        SKY.right.setPower(gamepad1.right_stick_x);

        //arms
        SKY.armV.setPower(-gamepad2.left_stick_y);
        SKY.armH.setPower(gamepad2.left_stick_x);
        SKY.armB1.setPower(gamepad2.right_stick_x);
        SKY.armB2.setPower(gamepad2.right_stick_y);

            if(gamepad2.a) {
                SKY.grabRight.setPosition(0);
                SKY.grabLeft.setPosition(1);
            }

            if(gamepad1.a) {
                SKY.grabHolder.setPosition(0);
            }
            if(gamepad2.b) {
                SKY.grabRight.setPosition(1);
                SKY.grabLeft.setPosition(0);
            }
            if(gamepad1.b) {
                SKY.grabHolder.setPosition(1);
            }

            if(gamepad2.y) {
                SKY.grabRight.setPosition(.55);
                SKY.grabLeft.setPosition(.45);
            }
            if(gamepad1.y) {
                SKY.grabHolder.setPosition(.5);

            }
            if(gamepad1.x) {
                SKY.grabRight.setPosition(1);
                SKY.grabLeft.setPosition(.5);
            }
            if(gamepad1.right_bumper){
                SKY.grabRightF.setPosition(1);
                SKY.grabLeftF.setPosition(0);
            }
            if(gamepad1.left_bumper){
             SKY.grabRightF.setPosition(0);
             SKY.grabLeftF.setPosition(1);
        }
            if(gamepad1.dpad_up){
                SKY.right.setPower(.5);
                SKY.left.setPower(.5);
            }
            if(gamepad1.dpad_down){
                SKY.right.setPower(-.5);
                SKY.left.setPower(-.5);
            }
            if(gamepad2.dpad_down){
                SKY.armV.setPower(-.5);
            }
            if(gamepad2.dpad_up){
                SKY.armV.setPower(.5);
            }
            if(gamepad2.dpad_left){
                SKY.armH.setPower(-.5);
            }
            if(gamepad2.dpad_right){
                SKY.armH.setPower(.5);
            }
            /*if (gamepad1.right_bumper) {
                SKY.smackServo.setPosition(.7);

            }*/
        }
        public void stop() {
        }
}
