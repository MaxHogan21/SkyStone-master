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
    This is the 7th iteration of the set of autonomous programs designed by 10101 Binary Bullets
    This program ultimately leaves the correct alliance color jewel on the platform, puts a pre-loaded
    Glyph into the correct VuMark Scanned Key Column and attempts to add additional Glyphs into a
    CryptoBox column. This program utilities 3 Modern Robotics Range Sensors(located on the front,
    left and rear sides of the robot), a REV Robotics Color Sensor, and VuMark sensing on the Moto G
    phone. This program is made for the Blue Alliance, right most stone from driver perspective. This
    program can score at the most 115 Autonomous points. This new program includes the added
    HardwareNatari Class, a new hardware class for the finalized robot.
 */

//@Autonomous(name="Blue_Straight_7")
public class Blue_Straight_7 extends LinearOpMode {

    HardwareNatari NATARI = new HardwareNatari();

    @Override
    public void runOpMode() throws InterruptedException {

        //Inherits hardware from Hardware class
        NATARI.init(hardwareMap);

        //Motor Direction
        NATARI.leftf.setDirection(DcMotor.Direction.REVERSE);
        NATARI.leftb.setDirection(DcMotor.Direction.REVERSE);

        //VuMark Trackables
        VuforiaTrackables relicTrackables = this.NATARI.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        // Wait for the game to start
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
                //^^Above pulls the hue values out of the first spot of the array HSV
                sleep(1500);
                NATARI.extarm.setPosition(1.0);
                sleep(500);
                NATARI.extarm.setPosition(.85);
                sleep(500);
                NATARI.jewelarm.setPosition(0.7);
                sleep(500);
                NATARI.extarm.setPosition(0.0);
                sleep(500);
                NATARI.jewelarm.setPosition(0.0);
                sleep(1500);

            } else {

                sleep(1500);
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

            if (opModeIsActive()&&vuMark == RelicRecoveryVuMark.LEFT) {

                //Move off Stone, Strafing
                rightStrafe(1.2, 1, 1750);
                //Pause to let the range sensor get a reading
                driveStop(1000);
                //Strafes until the set distance is greater than set value, positions robot to key column
                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.85);
                NATARI.leftb.setPower(.85);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 22) ;
                {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);

                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.85);
                NATARI.leftb.setPower(.85);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 22) ;
                {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);
                //Moves into key column until a front sensor reads less than 27 centimeters
                timelessMove(-.7);

                while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 27) {
                    telemetry.addData("front cm", "%.2f cm", NATARI.frontSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);
                /*
                Every Program has the same set of instructions despite which key column it goes to.
                instead of making changes to every single one, a function called afterLoop was created
                 */
                afterloop();

            } else if (opModeIsActive()&&vuMark == RelicRecoveryVuMark.CENTER) {

                //Move off Stone, Strafing
                rightStrafe(1.2, 1, 1750);
                //Pause to let the range sensor get a reading
                driveStop(1000);
                //Strafes until the set distance is greater than set value, positions robot to key column
                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.9);
                NATARI.leftb.setPower(.9);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 42) ;
                {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);

                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(.9);
                NATARI.leftb.setPower(.9);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 42) ;
                {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);
                //Moves into key column until a front sensor reads less than 27 centimeters
                timelessMove(-.7);

                while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 27) {
                    telemetry.addData("front cm", "%.2f cm", NATARI.frontSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);
                /*
                Every Program has the same set of instructions despite which key column it goes to.
                instead of making changes to every single one, a function called afterLoop was created
                 */
                afterloop();

            } else if (opModeIsActive()&&vuMark == RelicRecoveryVuMark.RIGHT) {

                //Move off Stone, Strafing
                rightStrafe(1.2, 1, 1750);
                //Pause to let the range sensor get a reading
                driveStop(1000);
                //Strafes until the set distance is greater than set value, positions robot to key column
                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(1);
                NATARI.leftb.setPower(1);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 57) ;
                {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);

                NATARI.leftf.setPower(-.7);
                NATARI.rightf.setPower(1);
                NATARI.leftb.setPower(1);
                NATARI.rightb.setPower(-.7);

                while (opModeIsActive() && NATARI.lsideSensor.getDistance(DistanceUnit.CM) < 57) ;
                {

                    telemetry.addData("left cm", "%.2f cm", NATARI.lsideSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);
                //Moves into key column until a front sensor reads less than 27 centimeters
                timelessMove(-.7);

                while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 27) {
                    telemetry.addData("front cm", "%.2f cm", NATARI.frontSensor.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }

                driveStop(500);
                /*
                Every Program has the same set of instructions despite which key column it goes to.
                instead of making changes to every single one, a function called afterLoop was created
                 */
                afterloop();
            }
        } while (opModeIsActive()&& NATARI.num > 10);
    }

    public void afterloop() {
        //Opens grabbers to drop pre-loaded Glyph
        NATARI.clawleft.setPosition(0.5);
        NATARI.clawright.setPosition(0.5);
        sleep(500);
        //Moves the central rack to push Glyph out of holding position
        NATARI.rack1.setPower(-1);
        sleep(700);

        NATARI.rack1.setPower(0);
        sleep(500);
        //Sets Grabbers to all the way open
        NATARI.clawleft.setPosition(0);
        NATARI.clawright.setPosition(1);
        sleep(500);
        //Moves robot BACKWARDS
        driveStraight(.8, 100);

        driveStop(100);
        //Moves Robot forwards, just for safety to make sure key Glyph is in spot
        driveStraight(-.5, 200);

        driveStop(100);
        //Moves robot Backwards
        driveStraight(.5, 500);

        driveStop(100);
        //Turns Robot around 180 degrees and faces Glyph pit
        rightTurn(1, 1600);

        driveStop(100);
        //Moves robot forward towards Glyph Pit
        driveStraight(-1, 900);

        driveStop(100);
        //Moves Central Rack out to Grab Glyph
        NATARI.rack1.setPower(-1);
        sleep(250);

        NATARI.rack1.setPower(0);
        sleep(500);
        //Grabs onto any Glyphs in the grabbers position
        NATARI.topleft.setPosition(0);
        NATARI.clawleft.setPosition(1);
        NATARI.topright.setPosition(1);
        NATARI.clawright.setPosition(0);
        sleep(500);
        /*
        Moves vertical rack up so if we do end up in the key column with the other pre-loaded
        Glyph we will stack on top of that glyph.
         */
        NATARI.climb.setPower(-1);
        sleep(700);
        NATARI.climb.setPower(0);
        //Moves Robot backwards
        driveStraight(.5, 1000);

        driveStop(100);

        //Turns Robot back towards Columns
        rightTurn(1, 1600);

        driveStop(200);
        //Just like before, moves robot until the front distance sensor sees 35 centimeters
        timelessMove(-.5);

        while (opModeIsActive() && NATARI.frontSensor.getDistance(DistanceUnit.CM) > 35) {
        }

        driveStop(100);
        //Drops the glyphs in spot
        NATARI.topleft.setPosition(1);
        NATARI.clawleft.setPosition(0);
        NATARI.topright.setPosition(0);
        NATARI.clawright.setPosition(1);
        //Pushes Glyph into Column
        driveStraight(-.5, 600);

        driveStop(200);
        //Backs up so not touching Glyph at the end of Match
        driveStraight(.5, 200);

        driveStop(200);
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