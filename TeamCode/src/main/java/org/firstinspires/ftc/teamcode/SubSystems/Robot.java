package org.firstinspires.ftc.teamcode.SubSystems;

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
    public Servo mainClawArm;
    public Servo mainClawRotation;
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

    // Declare game pad objects
    public double leftStickX;
    public double leftStickY;
    public double rightStickX;
    public double rightStickY;
    public boolean aButton;
    public boolean bButton;
    public boolean dPadUp;
    public boolean dPadDown;
    public boolean dPadLeft;
    public boolean dPadRight;
    public boolean bumperLeft;
    public boolean bumperRight;

    public double leftStickX2;
    public double leftStickY2;
    public double rightStickX2;
    public double rightStickY2;
    public boolean aButton2;
    public boolean bButton2;
    public boolean dPadUp2;
    public boolean dPadDown2;
    public boolean dPadLeft2;
    public boolean dPadRight2;
    public boolean bumperLeft2;
    public boolean bumperRight2;

    public boolean isaButtonPressedPrev =false;
    public boolean isbButtonPressedPrev = false;
    public boolean isxButtonPressedPrev =false;
    public boolean isyButtonPressedPrev = false;
    public boolean isaButton2PressedPrev = false;
    public boolean isbButton2PressedPrev = false;
    public boolean isxButton2PressedPrev = false;
    public boolean isyButton2PressedPrev = false;

    //Subsystems
    public Drive drive;
    public Control control;
    public Vision vision;

    public Robot(OpMode opMode, ElapsedTime timer){
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.timer = timer;
        init(0);
    }

    /**
     *
     * @param opMode
     * @param timer
     * @param visionMode
     *          o: no camera is initialized
     *          1: only armWebcam is initialized for OpenCV
     *          2: backWebcam is initialized for Vuforia
     *          3: backWebcam is initialized for Vuforia and frontWebcam is initialized for OpenCV
     *          4: armWebcam is initialized for OpenCV and frontWebcam is initialized for OpenCV
     */
    public Robot(OpMode opMode, ElapsedTime timer, int visionMode){
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.timer = timer;
        init(visionMode);
    }

    public Robot (){
        init(0);
    }

    public void init() {
        init(0);
    }

    public void init(int visionMode){
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
        mainClawArm = hardwareMap.servo.get("mA");
        mainClawRotation = hardwareMap.servo.get("mR");
        mainClaw = hardwareMap.servo.get("mC");
        csClaw = hardwareMap.servo.get("csC"); //capstone claw
        csArm = hardwareMap.servo.get("csA"); //capstone arm
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
        control = new Control(xRailWinch, armTilt, mainClaw, mainClawRotation, mainClawArm, csClaw, csArm, fClawL, fClawR, imu, timer, opMode);
        if (visionMode != 0) {
            vision = new Vision(hardwareMap, this, visionMode);
        }
    }

    public void initServosAuto() {
        this.control.closeMainClaw();
        this.control.closeCSClaw();
        this.control.setMainClawRotationDegrees(180.0);
        this.control.setMainClawArmDegrees(180.0);
        this.control.setCSClawArmDegrees(180.0);
        this.control.raiseClawsFromFoundation();
    }

    public void initServosTeleop() {
        this.control.closeMainClaw();
        this.control.closeCSClaw();
        this.control.setCSClawArmDegrees(180.0);
        this.control.raiseClawsFromFoundation();
    }

    public OpMode getOpmode(){
        return this.opMode;
    }

    public void getGamePadInputs() {
        leftStickX = opMode.gamepad1.left_stick_x;
        leftStickY = -opMode.gamepad1.left_stick_y;
        rightStickX = opMode.gamepad1.right_stick_x;
        rightStickY = opMode.gamepad1.right_stick_y;
        aButton = opMode.gamepad1.a;
        bButton = opMode.gamepad1.b;
        dPadUp = opMode.gamepad1.dpad_up;
        dPadDown = opMode.gamepad1.dpad_down;
        dPadLeft = opMode.gamepad1.dpad_left;
        dPadRight = opMode.gamepad1.dpad_right;
        bumperLeft = opMode.gamepad1.left_bumper;
        bumperRight = opMode.gamepad1.right_bumper;

        leftStickX2 = opMode.gamepad2.left_stick_x;
        leftStickY2 = -opMode.gamepad2.left_stick_y;
        rightStickX2 = opMode.gamepad2.right_stick_x;
        rightStickY2 = -opMode.gamepad2.right_stick_y;
        aButton2 = opMode.gamepad2.a;
        bButton2 = opMode.gamepad2.b;
        dPadUp2 = opMode.gamepad2.dpad_up;
        dPadDown2 = opMode.gamepad2.dpad_down;
        dPadLeft2 = opMode.gamepad2.dpad_left;
        dPadRight2 = opMode.gamepad2.dpad_right;
        bumperLeft2 = opMode.gamepad2.left_bumper;
        bumperRight2 = opMode.gamepad2.right_bumper;
    }

}

