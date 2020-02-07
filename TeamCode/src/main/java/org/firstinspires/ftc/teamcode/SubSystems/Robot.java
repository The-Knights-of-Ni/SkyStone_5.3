package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Drive;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.io.IOException;

/**
 * Created by AndrewC on 12/27/2019.
 */

public class Robot extends Subsystem {
    public String name;
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
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
    public BNO055IMU imu;
    private ColorSensor colorSensor;

    // Declare game pad objects
    public double leftStickX;
    public double leftStickY;
    public double rightStickX;
    public double rightStickY;
    public float triggerLeft;
    public float triggerRight;
    public boolean aButton = false;
    public boolean bButton = false;
    public boolean xButton = false;
    public boolean yButton = false;
    public boolean dPadUp = false;
    public boolean dPadDown = false;
    public boolean dPadLeft = false;
    public boolean dPadRight = false;
    public boolean bumperLeft = false;
    public boolean bumperRight = false;

    public double leftStickX2;
    public double leftStickY2;
    public double rightStickX2;
    public double rightStickY2;
    public float triggerLeft2;
    public float triggerRight2;
    public boolean aButton2 = false;
    public boolean bButton2 = false;
    public boolean xButton2 = false;
    public boolean yButton2 = false;
    public boolean dPadUp2 = false;
    public boolean dPadDown2 = false;
    public boolean dPadLeft2 = false;
    public boolean dPadRight2 = false;
    public boolean bumperLeft2 = false;
    public boolean bumperRight2 = false;

    public boolean isaButtonPressedPrev = false;
    public boolean isbButtonPressedPrev = false;
    public boolean isxButtonPressedPrev = false;
    public boolean isyButtonPressedPrev = false;
    public boolean isdPadUpPressedPrev = false;
    public boolean isdPadDownPressedPrev = false;
    public boolean isdPadLeftPressedPrev = false;
    public boolean isdPadRightPressedPrev = false;
    public boolean islBumperPressedPrev = false;
    public boolean isrBumperPressedPrev = false;
    public boolean isaButton2PressedPrev = false;
    public boolean isbButton2PressedPrev = false;
    public boolean isxButton2PressedPrev = false;
    public boolean isyButton2PressedPrev = false;
    public boolean isdPadUp2PressedPrev = false;
    public boolean isdPadDown2PressedPrev = false;
    public boolean isdPadLeft2PressedPrev = false;
    public boolean isdPadRight2PressedPrev = false;
    public boolean islBumper2PressedPrev = false;
    public boolean isrBumper2PressedPrev = false;

    private double joystickDeadZone = 0.1;

    //Subsystems
    public Drive drive;
    public Control control;
    public Vision vision;

    public Robot(LinearOpMode opMode, ElapsedTime timer) throws IOException {
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
    public Robot(LinearOpMode opMode, ElapsedTime timer, int visionMode) throws IOException {
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.timer = timer;
        init(visionMode);
    }

    public Robot () throws IOException {
        init(0);
    }

    public void init() throws IOException {
        init(0);
    }

    public void init(int visionMode) throws IOException {
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
        xRailWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        xRailWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xRailWinch.setTargetPosition(0);
        xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        xRailWinch.setPower(1.0);

        armTilt = (DcMotorEx) hardwareMap.dcMotor.get("tilt");
        armTilt.setDirection(DcMotorSimple.Direction.FORWARD);
        armTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTilt.setTargetPosition(0);
        armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTilt.setPower(1.0);

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

        opMode.telemetry.addData("Mode", " IMU initializing...");
        opMode.telemetry.update();
        imu.initialize(parameters);
        opMode.telemetry.addData("Mode", " IMU calibrating...");
        opMode.telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated())
        {
            opMode.sleep(50);
            opMode.idle();
        }

        opMode.telemetry.addData("Mode", " drive/control initializing...");
        opMode.telemetry.update();
        //Subsystems
        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, imu, opMode, timer);

//        drive.setMotorKp(10.0, 10.0, 10.0, 10.0);
//        drive.setMotorPID(5.0, 1.0, 1.0, 0.0);
//        drive.printMotorPIDCoefficients();
//        opMode.sleep(3000);

        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.initMaxVelocity();
//        drive.stop();
//        drive.printMotorPIDCoefficients();
//        opMode.sleep(2000);

        control = new Control(xRailWinch, armTilt, mainClaw, mainClawRotation, mainClawArm, csClaw, csArm, fClawL, fClawR, imu, opMode, timer);

        if (visionMode != 0) {
            opMode.telemetry.addData("Mode", " Camera initializing...");
            opMode.telemetry.update();
            vision = new Vision(hardwareMap, this, visionMode);
        }
    }

    public void initServosAuto() {
        this.control.closeCSClaw();
        this.control.setMainClawRotationDegrees(90.0);
        this.control.openMainClawWide();
        this.control.retractMainClawArm();
        this.control.retractCSClawArm();
        this.control.raiseClawsFromFoundation();
    }

    public void initServosTeleop() {
//        this.control.closeMainClaw();
        this.control.closeCSClaw();
//        this.control.setMainClawRotationDegrees(90.0);
        this.control.retractCSClawArm();
        this.control.raiseClawsFromFoundation();
    }

    public OpMode getOpmode(){
        return this.opMode;
    }

    public void getGamePadInputs() {
        isaButtonPressedPrev = aButton;
        isbButtonPressedPrev = bButton;
        isxButtonPressedPrev = xButton;
        isyButtonPressedPrev = yButton;
        isdPadUpPressedPrev = dPadUp;
        isdPadDownPressedPrev = dPadDown;
        isdPadLeftPressedPrev = dPadLeft;
        isdPadRightPressedPrev = dPadRight;
        islBumperPressedPrev = bumperLeft;
        isrBumperPressedPrev = bumperRight;
        leftStickX = joystickDeadzoneCorrection(opMode.gamepad1.left_stick_x);
        leftStickY = joystickDeadzoneCorrection(-opMode.gamepad1.left_stick_y);
        rightStickX = joystickDeadzoneCorrection(opMode.gamepad1.right_stick_x);
        rightStickY = joystickDeadzoneCorrection(opMode.gamepad1.right_stick_y);
        triggerLeft = opMode.gamepad1.left_trigger;
        triggerRight = opMode.gamepad1.right_trigger;
        aButton = opMode.gamepad1.a;
        bButton = opMode.gamepad1.b;
        xButton = opMode.gamepad1.x;
        yButton = opMode.gamepad1.y;
        dPadUp = opMode.gamepad1.dpad_up;
        dPadDown = opMode.gamepad1.dpad_down;
        dPadLeft = opMode.gamepad1.dpad_left;
        dPadRight = opMode.gamepad1.dpad_right;
        bumperLeft = opMode.gamepad1.left_bumper;
        bumperRight = opMode.gamepad1.right_bumper;

        isaButton2PressedPrev = aButton2;
        isbButton2PressedPrev = bButton2;
        isxButton2PressedPrev = xButton2;
        isyButton2PressedPrev = yButton2;
        isdPadUp2PressedPrev = dPadUp2;
        isdPadDown2PressedPrev = dPadDown2;
        isdPadLeft2PressedPrev = dPadLeft2;
        isdPadRight2PressedPrev = dPadRight2;
        islBumper2PressedPrev = bumperLeft2;
        isrBumper2PressedPrev = bumperRight2;
        leftStickX2 = joystickDeadzoneCorrection(opMode.gamepad2.left_stick_x);
        leftStickY2 = joystickDeadzoneCorrection(-opMode.gamepad2.left_stick_y);
        rightStickX2 = joystickDeadzoneCorrection(opMode.gamepad2.right_stick_x);
        rightStickY2 = joystickDeadzoneCorrection(-opMode.gamepad2.right_stick_y);
        triggerLeft2 = opMode.gamepad2.left_trigger;
        triggerRight2 = opMode.gamepad2.right_trigger;
        aButton2 = opMode.gamepad2.a;
        bButton2 = opMode.gamepad2.b;
        xButton2 = opMode.gamepad2.x;
        yButton2 = opMode.gamepad2.y;
        dPadUp2 = opMode.gamepad2.dpad_up;
        dPadDown2 = opMode.gamepad2.dpad_down;
        dPadLeft2 = opMode.gamepad2.dpad_left;
        dPadRight2 = opMode.gamepad2.dpad_right;
        bumperLeft2 = opMode.gamepad2.left_bumper;
        bumperRight2 = opMode.gamepad2.right_bumper;
    }

    public double joystickDeadzoneCorrection(double joystickInput) {
        double joystickOutput;
        if (joystickInput > joystickDeadZone) {
            joystickOutput = (joystickInput - joystickDeadZone) / (1.0-joystickDeadZone);
        }
        else if (joystickInput > -joystickDeadZone) {
            joystickOutput = 0.0;
        }
        else {
            joystickOutput = (joystickInput + joystickDeadZone) / (1.0-joystickDeadZone);
        }
        return joystickOutput;
    }


}

