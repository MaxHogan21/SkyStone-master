package org.firstinspires.ftc.teamcode.OldPrograms.RoverFiles;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.lang.annotation.ElementType;

public class ROVERHardware1 {

    //Hardware Map
    HardwareMap rrmap = null;

    //DC Motors
    public DcMotor leftf;
    public DcMotor leftb;
    public DcMotor rightf;
    public DcMotor rightb;
    public DcMotor climb;
    public DcMotor ext;
    public DcMotor lift;

    //Servo Motors
    public Servo grab;
    public CRServo sweep;
    public Servo dump;

    //Sensors
    public DistanceSensor sensorColor;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public DistanceSensor rev2M;
    public DigitalChannel magnentLimit;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    public ElapsedTime timer = new ElapsedTime();

    //Constants
    int sleep = 0;
    int threshold = 4000;
    int one = 1;
    public double globalAngle;
    public double power = .5;
    public double floor = 20;


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    private static final String VUFORIA_KEY = "ASmWZqr/////AAAAGTT1hSkQHkAdnfLu4EGOvd4Viq8whTdXVtvPRsoUkuVOHtgPaMjgiL3GLlbIQPIGt9JPuLd/gTNLgwOzUFdcad4CleqkXd0sDGEC7oay+uagw2fMsfOHyga/a6NFz+89V/WX2sg/dgGJXCWSArfvb4xMjXx9fY91gFJ9sXsunNujGB4xUKpBUdF2tGMssErR84JpmyPc5t9FTt9haog15732zHWQvG0UV2sgzRF9EP68DQtYqHYxOhejX2l0lvidMyihjbGG6v6br8JE2Ig8hLjdZ7Iq5PfZxcT/QRi4kpauguz6eyKXjtI6/la+2Lz5EHqGz+taC1dvnvDhlA2IeSvdcF6lVo/UpaXCR2nGWGZI";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public ROVERHardware1() {

    }

    public void init(HardwareMap bbMap) {

        rrmap = bbMap;
        leftf = bbMap.dcMotor.get("left_front_motor");
        leftb = bbMap.dcMotor.get("left_back_motor");
        rightf = bbMap.dcMotor.get("right_front_motor");
        rightb = bbMap.dcMotor.get("right_back_motor");
        climb = bbMap.dcMotor.get("climb_motor");
        ext = bbMap.dcMotor.get("ext_motor");
        lift = bbMap.dcMotor.get("lift_motor");
        sweep = bbMap.crservo.get("collect_servo");
        grab = bbMap.servo.get("grab_servo");
        dump = bbMap.servo.get("dump_servo");
        sensorColor = bbMap.get(DistanceSensor.class, "sensor_color_distance");
        rangeSensor = bbMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        magnentLimit = bbMap.get(DigitalChannel.class, "magnent_limit");
        rev2M = bbMap.get(DistanceSensor.class, "rev_distance");

    }
}
