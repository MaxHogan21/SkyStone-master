package Worlds;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SKYHardware {

    HardwareMap skyhw = null;

    // DC motors
    public DcMotor leftF;
    public DcMotor rightF;
    public DcMotor leftB;
    public DcMotor rightB;
    public DcMotor armV;
    public DcMotor armH;
    public DcMotor armB1;
    public DcMotor armB2;

    // servos
    public Servo grabRight;
    public Servo grabLeft;
    public Servo grabHolder;
    public Servo grabLeftF;
    public Servo grabRightF;
    public CRServo parkServo;

    final String vuforiaKey = "AQMXapn/////AAABmUkIKliu8UhAlsC0hI8AZ/gGzHN693N16RDIV6KvJMcygzolaMYuceUhBHEFuw9JwHBpBSS2OV/BEczUwrgYp9iMPev1ooBl10M89qxmmps38aXL7YycUEe3FTH/0YnvFmPCqUc60Hr0rpAgYqcbmKNfGPF7GCVYsHDGTjUUJAepX5HiX1UUES01Wji5ZArDu9A3oTSMvjSVULFB6wLXRKK8Qk8p/sh3NZsg11NtgjePsUckyvJXTVxTaRwltAWBh9eLZsMwHsZD5pcUSsJwXQFIqGwYE7T7fTMGhPZw/V1bsKTzp7rw5ErPbeBvLUzyHe9DlIyLbJqQ1pIoF9UP+PbQgz3HHf0F7bsKpc3EGa0l";
    public double power;

    public SKYHardware() { }

    public void init(HardwareMap sshw) {
        leftB = sshw.dcMotor.get("leftB");
        rightB = sshw.dcMotor.get("rightB");
        rightF = sshw.dcMotor.get("rightF");
        leftF = sshw.dcMotor.get("leftF");
        armV = sshw.dcMotor.get("armV");
        armH = sshw.dcMotor.get("armH");
        armB1 = sshw.dcMotor.get("armB1");
        armB2 = sshw.dcMotor.get("armB2");
        grabRight = sshw.servo.get("grabRight");
        grabLeft = sshw.servo.get("grabLeft");
        grabHolder = sshw.servo.get("grabHolder");
        grabLeftF = sshw.servo.get("grabLeftF");
        grabRightF = sshw.servo.get("grabRightF");
        parkServo = sshw.crservo.get("parkServo");
        rightF.setDirection(DcMotor.Direction.REVERSE);
        leftF.setDirection(DcMotor.Direction.REVERSE);
        armB2.setDirection(DcMotor.Direction.REVERSE);
        armV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
