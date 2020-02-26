package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
    This is the 4th iteration of the set of autonomous programs designed by 10101 Binary Bullets
    This program ultimately leaves the correct alliance color jewel on the platform, puts a pre-loaded
    Glyph into the correct VuMark Scanned Key Column and attempts to add additional Glyphs into a
    CryptoBox column. This program utilities 3 Modern Robotics Range Sensors(located on the front,
    left and rear sides of the robot), a REV Robotics Color Sensor, and VuMark sensing on the Moto G
    phone. This program is made for the Blue Alliance, left most stone from driver perspective. This
    program can score at the most 115 Autonomous points. This new program includes the added
    HardwareNatari Class, a new hardware class for the finalized robot.
 */

//@Autonomous(name = "Blue_Turn_4")
public class Blue_Turn_4 extends LinearOpMode {

    //Inherits hardware from Hardware class
    HardwareNatari NATARI = new HardwareNatari();

    public void runOpMode() throws InterruptedException {

        //Hardware Map
        NATARI.init(hardwareMap);

        //VuMark Trackables
        VuforiaTrackables relicTrackables = this.NATARI.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        //Motor Direction
        NATARI.leftf.setDirection(DcMotor.Direction.REVERSE);
        NATARI.leftb.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //Moves Arm to knock Jewel into Position
        NATARI.jewelarm.setPosition(0.8);
        sleep(1500);
        NATARI.extarm.setPosition(.85);
        sleep(500);
        NATARI.jewelarm.setPosition(1.0);
        sleep(500);

        //Imports Template for VuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        //Instead of Color, uses the Hue value for more accurate measurements, 300>Red<50, 300<Blue>50
        //Imports Array to go from Red Green Blue to Hue Saturation Value
        Color.RGBToHSV(NATARI.revsensor.red() * 8, NATARI.revsensor.green() * 8, NATARI.revsensor.blue() * 8, NATARI.hsvValues);
        do {
            if (NATARI.hsvValues[0] > 300 || NATARI.hsvValues[0] < 50) {

                sleep(1000);
                NATARI.extarm.setPosition(1.0);
                sleep(1200);
                NATARI.extarm.setPosition(.8);
                sleep(500);
                NATARI.jewelarm.setPosition(0.7);
                sleep(500);
                NATARI.extarm.setPosition(0.0);
                sleep(500);
                NATARI.jewelarm.setPosition(0.0);
                sleep(1500);

            } else {

                sleep(1000);
                NATARI.extarm.setPosition(0.0);
                sleep(500);
                NATARI.jewelarm.setPosition(0.0);
                sleep(1500);

            }
        } while (opModeIsActive() && NATARI.num > 10);
        /*
        Easy way to trick the program to run an if/else statement once and continue with the rest
        of the program using a do-while loop. The do part always executes once before checking
        the conditions of the loop. Therefore, though the while loop is false, it still runs the
        if/else statement at least once. This discovery makes coding loops into linear Op Modes
        much easier.
        */
        do {

            if (vuMark == RelicRecoveryVuMark.LEFT) {

                //Move off Stone, Strafing
                rightStrafe(.9, .7, 1800);

                driveStop(500);
                //Turns so Robot is now facing the CryptoBox
                rightTurn(1,750);
                //Pause to let the range sensor get a reading
                driveStop(1000);
                //Strafes until the set distance is greater than set value, positions robot to key column
                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.85);
                NATARI.leftb.setPower(.85);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 46); {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                sleep(500);

                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.85);
                NATARI.leftb.setPower(.85);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 46); {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                //Moves into key column until a front sensor reads less than 27 centimeters
                timelessMove(-.7);

                while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 27){
                    telemetry.addData("front cm", "%.2f cm", NATARI.frontSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                /*
                Every Program has the same set of instructions despite which key column it goes to.
                instead of making changes to every single one, a function called afterLoop was created
                 */
                afterloop();

            } else if (vuMark == RelicRecoveryVuMark.CENTER) {

                //Move off Stone, Strafing
                rightStrafe(.9, .7, 1900);

                driveStop(500);
                //Turns so Robot is now facing the CryptoBox
                rightTurn(1,750);
                //Pause to let the range sensor get a reading
                driveStop(1000);
                //Strafes until the set distance is greater than set value, positions robot to key column
                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.9);
                NATARI.leftb.setPower(.9);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 63); {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                sleep(500);

                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.9);
                NATARI.leftb.setPower(.9);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 63); {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);

                //Moves into key column until a front sensor reads less than 27 centimeters
                timelessMove(-.7);

                while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 27){
                    telemetry.addData("front cm", "%.2f cm", NATARI.frontSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                /*
                Every Program has the same set of instructions despite which key column it goes to.
                instead of making changes to every single one, a function called afterLoop was created
                 */
                afterloop();

            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {

                //Move off Stone, Strafing
                rightStrafe(.9, .7, 1900);

                driveStop(500);
                //Turns so Robot is now facing the CryptoBox
                rightTurn(1,750);
                //Pause to let the range sensor get a reading
                driveStop(1000);
                //Strafes until the set distance is greater than set value, positions robot to key column
                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.9);
                NATARI.leftb.setPower(.9);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 78); {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                sleep(500);

                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.9);
                NATARI.leftb.setPower(.9);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 78); {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                sleep(500);
                //Moves into key column until a front sensor reads less than 27 centimeters
                timelessMove(-.7);

                while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 27){
                    telemetry.addData("front cm", "%.2f cm", NATARI.frontSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(200);
                /*
                Every Program has the same set of instructions despite which key column it goes to.
                instead of making changes to every single one, a function called afterLoop was created
                 */
                afterloop();
            }
        }while(NATARI.num > 10);
    }



    public void afterloop()
    {
        NATARI.clawleft.setPosition(0.5);
        NATARI.clawright.setPosition(0.5);
        sleep(500);

        NATARI.rack1.setPower(-1);
        sleep(700);

        NATARI.rack1.setPower(0);
        sleep(100);

        NATARI.clawleft.setPosition(0);
        NATARI.clawright.setPosition(1);
        sleep(250);

        driveStraight(.8,500);

        driveStop(100);

        driveStraight(-.5,200);

        driveStop(100);

        driveStraight(.5,300);

        driveStop(200);

        NATARI.leftf.setPower(-.7);
        NATARI.rightf.setPower(.9);
        NATARI.leftb.setPower(.9);
        NATARI.rightb.setPower(-.7);

        while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 88); {

        telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        }

        driveStop(100);

        rightTurn(1,1400);

        driveStop(100);

        driveStraight(-1, 1150);

        driveStop(100);

        NATARI.rack1.setPower(-1);
        sleep(250);

        NATARI.rack1.setPower(0);
        sleep(500);

        NATARI.topleft.setPosition(0);
        NATARI.clawleft.setPosition(1);
        NATARI.topright.setPosition(1);
        NATARI.clawright.setPosition(0);
        sleep(500);

        NATARI.climb.setPower(-1);
        sleep(700);
        NATARI.climb.setPower(0);

        rightTurn(1,1500);

        driveStop(200);

        timelessMove(-.5);

        while(opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 35){}

         driveStop(100);

        driveStraight(-.5,400);

        driveStop(100);

        NATARI.topleft.setPosition(1);
        NATARI.clawleft.setPosition(0);
        NATARI.topright.setPosition(0);
        NATARI.clawright.setPosition(1);

        driveStraight(.5, 200);

        driveStop(100);

    }
    public void driveStraight(double power, int Sleep) {
        NATARI.leftf.setPower(power);
        NATARI.rightf.setPower(power);
        NATARI.leftb.setPower(power);
        NATARI.rightb.setPower(power);
        sleep(Sleep);
        if(!opModeIsActive()) {
            return;
        }
    }

    public void driveStop(int Sleep) {
        NATARI.leftf.setPower(0);
        NATARI.rightf.setPower(0);
        NATARI.leftb.setPower(0);
        NATARI.rightb.setPower(0);
        sleep(Sleep);
        if(!opModeIsActive()) {
            return;
        }
    }

    public void rightTurn(int value, int Sleep) {
        NATARI.leftf.setPower(-value);
        NATARI.rightf.setPower(value);
        NATARI.leftb.setPower(-value);
        NATARI.rightb.setPower(value);
        sleep(Sleep);
        if(!opModeIsActive()) {
            return;
        }
    }

    public void rightStrafe(double high, double low, int Sleep) {
        NATARI.leftf.setPower(-low);
        NATARI.rightf.setPower(high);
        NATARI.leftb.setPower(high);
        NATARI.rightb.setPower(-low);
        sleep(Sleep);
        if(!opModeIsActive()) {
            return;
        }
    }

    public void leftTurn(int value, int Sleep) {
        NATARI.leftf.setPower(value);
        NATARI.rightf.setPower(-value);
        NATARI.leftb.setPower(value);
        NATARI.rightb.setPower(-value);
        sleep(Sleep);
        if(!opModeIsActive()) {
            return;
        }
    }

    public void leftStrafe(double high, double low, int Sleep) {
        NATARI.leftf.setPower(low);
        NATARI.rightf.setPower(-high);
        NATARI.leftb.setPower(-high);
        NATARI.rightb.setPower(low);
        sleep(Sleep);
        if(!opModeIsActive()) {
            return;
        }
    }

    public void timelessMove(double value) {
        NATARI.leftf.setPower(value);
        NATARI.rightf.setPower(value);
        NATARI.leftb.setPower(value);
        NATARI.rightb.setPower(value);
        if(!opModeIsActive()) {
            return;
        }
    }

}