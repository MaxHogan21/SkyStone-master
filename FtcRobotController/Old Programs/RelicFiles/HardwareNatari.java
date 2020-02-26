package org.firstinspires.ftc.teamcode.RelicFiles;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
This is the 1st iteration of our robot hardware class. This ncludes all of our hardware
and configurations for easier program acsess. Why is it called HardwareNatari? Natari
is the name of our robot, it stands for Nintendo and Atari because one of our themes this
season is retro video games.
 */

public class HardwareNatari {
    //DcMotors
    public DcMotor leftf;
    public DcMotor leftb;
    public DcMotor rightf;
    public DcMotor rightb;
    public DcMotor climb;
    public DcMotor rack1;
    //public DcMotor rack2;

    //Servo
    public Servo clawright;
    public Servo clawleft;
    public Servo topright;
    public Servo topleft;
    public Servo jewelarm;
    public Servo extarm;

    //Sensors
    public ColorSensor revsensor;
    public ModernRoboticsI2cRangeSensor rsideSensor;
    public ModernRoboticsI2cRangeSensor lsideSensor;
    public ModernRoboticsI2cRangeSensor frontSensor;

    //Constant for loops
    int num = 0;
    int varCount = 0;

    //HSV Values for
    float hsvValues[] = {0F, 0F, 0F};

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //Local OpMode
    HardwareMap hwMap = null;
    private ElapsedTime tme = new ElapsedTime();

    Orientation angles;

    // Constructor
    public HardwareNatari() {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //IMU Initialization

        //Devices to Config
        leftf = hwMap.dcMotor.get("left_front_motor");
        leftb = hwMap.dcMotor.get("left_back_motor");
        rightf = hwMap.dcMotor.get("right_front_motor");
        rightb = hwMap.dcMotor.get("right_back_motor");
        rack1 = hwMap.dcMotor.get("rack_1");
        //rack2 = hwMap.dcMotor.get("rack_2");
        jewelarm = hwMap.servo.get("jewel_motor");
        topleft = hwMap.servo.get("top_Left_motor");
        topright = hwMap.servo.get("top_Right_motor");
        clawright = hwMap.servo.get("claw_Right_motor");
        clawleft = hwMap.servo.get("claw_Left_motor");
        extarm = hwMap.servo.get("ext_arm");
        climb = hwMap.dcMotor.get("vertical_climb");
        revsensor = hwMap.colorSensor.get("sensor_color_distance");
        rsideSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "right_range");
        lsideSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        frontSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "front_range");

        //Vuforia Initialization
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASmWZqr/////AAAAGTT1hSkQHkAdnfLu4EGOvd4Viq8whTdXVtvPRsoUkuVOHtgPaMjgiL3GLlbIQPIGt9JPuLd/gTNLgwOzUFdcad4CleqkXd0sDGEC7oay+uagw2fMsfOHyga/a6NFz+89V/WX2sg/dgGJXCWSArfvb4xMjXx9fY91gFJ9sXsunNujGB4xUKpBUdF2tGMssErR84JpmyPc5t9FTt9haog15732zHWQvG0UV2sgzRF9EP68DQtYqHYxOhejX2l0lvidMyihjbGG6v6br8JE2Ig8hLjdZ7Iq5PfZxcT/QRi4kpauguz6eyKXjtI6/la+2Lz5EHqGz+taC1dvnvDhlA2IeSvdcF6lVo/UpaXCR2nGWGZI";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Set-with
        leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Several Functions that are imported every program

}





