package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ROVERHardware1;

@Autonomous(name="LiftTest")
@Disabled
public class LiftTest extends LinearOpMode {

    ROVERHardware1 ROVER = new ROVERHardware1();

    public void runOpMode() throws InterruptedException {

        ROVER.init(hardwareMap);

        waitForStart();

        sleep(500);
        ROVER.ext.setPower(1);
        sleep(2000);
        ROVER.lift.setPower(1);
        sleep(1000);



    }
}
