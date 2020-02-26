package Old_not_useful;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Tele", group = "Functional")
@Disabled

public class firstTry extends OpMode {
    //motors
    DcMotor left;
    DcMotor right;
    DcMotor armV;
    DcMotor armH;
    DcMotor armB1;
    DcMotor armB2;
    Servo grabRight;
    Servo grabLeft;
    Servo grabHolder;
    Servo smackServo;




    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        armV = hardwareMap.dcMotor.get("armV");
        armH = hardwareMap.dcMotor.get("armH");
        armB1 = hardwareMap.dcMotor.get("armB1");
        armB2 = hardwareMap.dcMotor.get("armB2");
        grabRight = hardwareMap.servo.get("grabRight");
        grabLeft = hardwareMap.servo.get("grabLeft");
        grabHolder = hardwareMap.servo.get("grabHolder");
        smackServo = hardwareMap.servo.get("smackServo");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        /*oriented in front of arm lift



        going forwards/backwards */
        left.setPower(gamepad1.left_stick_y);
        right.setPower(-gamepad1.left_stick_y);

        //turning
        left.setPower(gamepad1.right_stick_x);
        right.setPower(gamepad1.right_stick_x);

        //arms
        armV.setPower(-gamepad2.left_stick_y);
        armH.setPower(gamepad2.left_stick_x);
        armB1.setPower(gamepad2.right_stick_x);
        armB2.setPower(gamepad2.right_stick_y);


        //servos
        if (gamepad2.a) {
                grabRight.setPosition(0);
                grabLeft.setPosition(1);
            }

        if (gamepad1.a) {
                grabHolder.setPosition(0);
            }
        if (gamepad2.b) {
                grabRight.setPosition(1);
                grabLeft.setPosition(0);
            }
        if (gamepad1.b) {
                grabHolder.setPosition(1);
            }

        if (gamepad2.y) {
                grabRight.setPosition(.55);
                grabLeft.setPosition(.45);
            }
        if (gamepad1.y) {
                grabHolder.setPosition(.5);

            }
        if (gamepad1.x) {
                grabRight.setPosition(1);
                grabLeft.setPosition(.5);
            }
        if(gamepad1.right_bumper){
            smackServo.setPosition(.7);

        }
        }
    }












