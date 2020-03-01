package Worlds;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
//@Disabled
public class Tele_Worlds extends OpMode {

    // Inherits hardware class
    private SKYHardware SKY = new SKYHardware();

    // This is the same sleep method that is used in autonomous programs. We use it for delays in
    // the main loop to prevent the program from thinking a button was pressed multiple times
    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    // Variables used in controls below
    private boolean rServoWasPressed = false;
    private boolean lServoWasPressed = false;

    public void init() {
        // Initializes hardware map for the phones
        SKY.init(hardwareMap);
    }
    public void loop() {

        // Telemetry statements for Driver Station

        // Shows how long we have been running teleop for practice and in case the time isn't
        // Visible during a match
        telemetry.addData("Runtime: ",(int) getRuntime());



        // Primary driver's controls

        // Driving

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        SKY.leftF.setPower(v1);
        SKY.rightF.setPower(v2);
        SKY.leftB.setPower(v3);
        SKY.rightB.setPower(v4);

        telemetry.addData("leftF power: ", v1);
        telemetry.addData("rightF power: ", v2);
        telemetry.addData("leftB power: ", v3);
        telemetry.addData("rightB power: ", v4);

        // Controls for our servo on the back of our robot to grab the foundation
        if (gamepad1.a) {
            SKY.grabHolder.setPosition(0);
        }
        if (gamepad1.b) {
            SKY.grabHolder.setPosition(1);
        }
        if (gamepad1.y) {
            SKY.grabHolder.setPosition(.5);
        }
        // Control that helps orient stones that are not perfectly aligned using our grabbers
        if (gamepad1.x) {
            SKY.grabRight.setPosition(1);
            SKY.grabLeft.setPosition(.5);
        }
        // Controls for our front servos to grab the foundation
        if (gamepad1.right_bumper){
            if (!rServoWasPressed){
                SKY.grabRightF.setPosition(0);
                rServoWasPressed = true;
                sleep(300);
            }
            else{
                SKY.grabRightF.setPosition(1);
                rServoWasPressed = false;
                sleep(300);
            }
        }
        if (gamepad1.left_bumper) {
            if (!lServoWasPressed){
                SKY.grabLeftF.setPosition(1);
                lServoWasPressed = true;
                sleep(300);
            }
            else{
                SKY.grabLeftF.setPosition(0);
                lServoWasPressed = false;
                sleep(300);
            }
        }
        // Moves our CR servo that shoots out our tape measure, which is one of our parking options
        if(gamepad1.dpad_right){
            SKY.parkServo.setPower(-.5);
        } else if (gamepad1.dpad_left){
            SKY.parkServo.setPower(.5);
        } else  {
            SKY.parkServo.setPower(0);
        }
        // Moves our drive train slower for more precise movements
        /*if(gamepad1.dpad_up){
            SKY.left.setPower(.5);
            SKY.right.setPower(.5);
        }
        if(gamepad1.dpad_down){
            SKY.right.setPower(-.5);
            SKY.left.setPower(-.5);
        }*/
        // Accessory driver's controls

        // Moves our main building arm
        SKY.armV.setPower(gamepad2.left_stick_y);
        // Moves our back arm that we use in autonomous in case there was an issue
        SKY.armB1.setPower(gamepad2.right_stick_x);
        SKY.armB2.setPower(gamepad2.right_stick_y);

        // Controls that move our grabbers
        if (gamepad2.a) {
            SKY.grabRight.setPosition(0);
            SKY.grabLeft.setPosition(1);
        }
        if (gamepad2.b) {
            SKY.grabRight.setPosition(1);
            SKY.grabLeft.setPosition(0);
        }
        if (gamepad2.y) {
            SKY.grabRight.setPosition(.55);
            SKY.grabLeft.setPosition(.45);
        }
        // The first two if statements increase or decrease our arm increment variable value.
        // This allows us to keep track of what level our
        // tower is at, allowing our accessory driver to be able to push a button and our main arm
        // will move to the appropriate level, using the bottom if statement
        // Allows us to move our main arm slower for more precise movements
        if (gamepad2.dpad_down) {
            SKY.armV.setPower(-.5);
        }
        if (gamepad2.dpad_up) {
            SKY.armV.setPower(.5);
        }
        // Allows us to move our linear actuator so we can adjust the horizontal position of stones
        if (gamepad2.dpad_left) {
            SKY.armH.setPower(-1);
        }
        else if (gamepad2.dpad_right) {
            SKY.armH.setPower(1);
        }
        else {
            SKY.armH.setPower(0);
        }
    }
    // Makes sure the program stops completely
    public void stop () {
    }
}