package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

/**
 * Created by AndrewC on 12/27/2019.
 */

public class Robot {
    public String name;
    private HardwareMap hardwareMap;
    private OpMode opMode;
    public ElapsedTime timer;

    //DC Motors
    public DcMotorEx frontLeftDriveMotor;
    public DcMotorEx frontRightDriveMotor;
    public DcMotorEx rearRightDriveMotor;
    public DcMotorEx rearLeftDriveMotor;
    public DcMotorEx xRailWinch;
    public DcMotorEx armTilt;

    //Servos
    public Servo mainArm;
    public Servo mainRotation;
    public Servo mainClaw; //0
    public Servo csClaw; //capstone claw
    public Servo csArm; //capstone arm
    public Servo fClawL; //foundationClawLeft
    public Servo fClawR; // foundationClawRight

    /**
     * HUB2
     * fl   0
     * fr   1
     * bl   2
     * br   3
     *
     * mA (mainArm)         2
     * mR (mainRotation)    1
     * mC (mainClaw)        0
     * csC (csClaw)         3
     * csA (csArm)          4
     * --------------------
     * HUB1
     * tilt     0
     * winch    1
     *
     * fL (fClawL)          2
     * fR (fClawR)          1
     */

    //Sensors
    private BNO055IMU imu;
    private ColorSensor colorSensor;

    //Subsystems
    public Drive drive;
    public Vision vision;

    public Robot(OpMode opMode, ElapsedTime timer){
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.timer = timer;
        init();
    }

    public Robot (){
        init();
    }
    public void init(){
        //DC Motors
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");

        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        xRailWinch = (DcMotorEx) hardwareMap.dcMotor.get("winch");
        armTilt = (DcMotorEx) hardwareMap.dcMotor.get("tilt");

        xRailWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        armTilt.setDirection(DcMotorSimple.Direction.REVERSE);



        xRailWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
        mainArm = hardwareMap.servo.get("mA");
        mainRotation = hardwareMap.servo.get("mR");
        mainClaw = hardwareMap.servo.get("mC");
//        csClaw = hardwareMap.servo.get("csC"); //capstone claw
//        csArm = hardwareMap.servo.get("csA"); //capstone arm
        fClawL = hardwareMap.servo.get("fL");
        fClawR = hardwareMap.servo.get("fR");

//        // Set servo scale ranges
//        mainArm.scaleRange(0,1);
//        mainRotation.scaleRange(0,1);
//        mainClaw.scaleRange(0,1);
//        fClawL.scaleRange(0.36,0.8);
//        fClawR.scaleRange(0.11,0.63);

        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        colorSensor = hardwareMap.colorSensor.get("color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu.initialize(parameters);

        //Subsystems
        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, imu, timer, opMode);
        vision = new Vision(hardwareMap, this);
    }

    public OpMode getOpmode(){
        return this.opMode;
    }
}

