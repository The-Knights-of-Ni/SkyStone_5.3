package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by tarunsingh on 9/24/17.
 */

@TeleOp(name = "TeleopMark1")
public class TeleopMark1 extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    double leftStickX;
    double leftStickY;
    double rightStickX;
    boolean aButton;
    boolean bButton;
    boolean dPadUp;
    boolean dPadDown;
    boolean dPadLeft;
    boolean dPadRight;

    double leftStickX2;
    double leftStickY2;
    double rightStickX2;
    double rightStickY2;
    boolean aButton2;
    boolean bButton2;
    boolean dPadUp2;
    boolean dPadDown2;
    boolean dPadLeft2;
    boolean dPadRight2;
    boolean bumperLeft2;
    boolean bumperRight2;

    int winchCurrentPosition = 0;
    int winchTargetPositionCurrent = 0;
    int winchTargetPositionPre = 0;
    int winchPosError = 400;
    int winchMax = 8410;

    int tiltCurrentPosition = 0;
    int tiltTargetPositionCurrent = 0;
    int tiltTargetPositionPre = 0;
    int tiltPosError = 300;
    int tiltMax = 1200;

    double timePre;
    double timeCurrent;
    double winchSpeed;
    double tiltSpeed;
    double deltaT;
    double winchIncrement = 0;
    double tiltIncrement = 0;
    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();
        telemetry.clearAll();
        while(opModeIsActive()) {
            //Get gamepad inputs
            leftStickX = gamepad1.left_stick_x;
            leftStickY = -gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;
            aButton = gamepad1.a;
            bButton = gamepad1.b;
            dPadUp = gamepad1.dpad_up;
            dPadDown = gamepad1.dpad_down;
            dPadLeft = gamepad1.dpad_left;
            dPadRight = gamepad1.dpad_right;

            leftStickX2 = gamepad2.left_stick_x;
            leftStickY2 = -gamepad2.left_stick_y;
            rightStickX2 = gamepad2.right_stick_x;
            rightStickY2 = -gamepad2.right_stick_y;
            aButton2 = gamepad2.a;
            bButton2 = gamepad2.b;
            dPadUp2 = gamepad2.dpad_up;
            dPadDown2 = gamepad2.dpad_down;
            dPadLeft2 = gamepad2.dpad_left;
            dPadRight2 = gamepad2.dpad_right;
            bumperLeft2 = gamepad2.left_bumper;
            bumperRight2 = gamepad2.right_bumper;


            timeCurrent = timer.nanoseconds();

            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
            robot.rearLeftDriveMotor.setPower(motorPowers[0]);
            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
            robot.rearRightDriveMotor.setPower(motorPowers[2]);
            robot.frontRightDriveMotor.setPower(motorPowers[3]);

            deltaT = timeCurrent - timePre;

            //Winch
            //930mm, 8400 encoder count
            if((leftStickY2 > 0.5)){
                robot.xRailWinch.setPower(1.0);
            }
            else if(leftStickY2 > 0.1){
                robot.xRailWinch.setPower(0.5);
            }
            else if(leftStickY2 > -0.5){
                robot.xRailWinch.setPower(0.3);
            }
            else {
                robot.xRailWinch.setPower(0.7);
            }

            if(leftStickY2 >= 0.1){
                winchSpeed = (leftStickY2 - 0.1) * (robot.drive.getWinchMaxSpeedTickPerSec() / 0.9);
            }
            else if(leftStickY2  <= -0.1){
                winchSpeed = (leftStickY2 + 0.1) * (robot.drive.getWinchMaxSpeedTickPerSec() / 0.9);
            }
            else{
                winchSpeed = 0.0;
            }
            winchCurrentPosition = robot.xRailWinch.getCurrentPosition();
            winchIncrement = (winchSpeed * deltaT) / Math.pow(10.0,9);
            winchTargetPositionCurrent = (int) (winchTargetPositionPre + winchIncrement);
            if((winchTargetPositionCurrent <= winchMax) && (winchTargetPositionCurrent >= 0)
                    && ( Math.abs(winchTargetPositionCurrent - winchCurrentPosition) < winchPosError)){
                robot.xRailWinch.setTargetPosition(winchTargetPositionCurrent);
                winchTargetPositionPre = winchTargetPositionCurrent;
            }

            //Tilt
            if((rightStickY2 > 0.5)){
                robot.armTilt.setPower(1.0);
            }
            else if(rightStickY2 > 0.1){
                robot.armTilt.setPower(0.7);
            }
            else if(rightStickY2 > -0.5){
                robot.armTilt.setPower(1.0);
            }
            else{
                robot.armTilt.setPower(1.0);
            }

            if(rightStickY2 >= 0.1){
                tiltSpeed = (rightStickY2 - 0.1) * (robot.drive.getTiltMaxSpeedTickPerSec() / 0.9);
            }
            else if(rightStickY2  <= -0.1){
                tiltSpeed = (rightStickY2 + 0.1) * (robot.drive.getTiltMaxSpeedTickPerSec() / 0.9);
            }
            else{
                tiltSpeed = 0.0;
            }
            tiltCurrentPosition = robot.armTilt.getCurrentPosition();
            tiltIncrement = (tiltSpeed * deltaT) / Math.pow(10.0,9);
            tiltTargetPositionCurrent = (int) (tiltTargetPositionPre + tiltIncrement);
            if((tiltTargetPositionCurrent <= tiltMax) && (tiltTargetPositionCurrent >= 0)
                    && ( Math.abs(tiltTargetPositionCurrent - tiltCurrentPosition) < tiltPosError)){
                robot.armTilt.setTargetPosition(tiltTargetPositionCurrent);
                tiltTargetPositionPre = tiltTargetPositionCurrent;
            }

            telemetry.addData("Left Stick Y2", leftStickY2);
            telemetry.addData("Right Stick Y2", rightStickY2);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("deltaT", deltaT);

//            telemetry.addData("currentPosWinch", winchCurrentPosition);
//            telemetry.addData("targetPosWinch", winchTargetPositionCurrent);
//            telemetry.addData("incrementWinch", winchIncrement);

            telemetry.addData("currentPosTilt", tiltCurrentPosition);
            telemetry.addData("targetPosTilt", tiltTargetPositionCurrent);
            telemetry.addData("incrementTilt", tiltIncrement);


//            telemetry.addData("", "");
//            telemetry.addData("Left Rear Power", robot.rearLeftDriveMotor.getPower());
//            telemetry.addData("Left Front Power", robot.frontLeftDriveMotor.getPower());
//            telemetry.addData("Right Rear Power", robot.rearRightDriveMotor.getPower());
//            telemetry.addData("Right Front Power", robot.frontRightDriveMotor.getPower());
            telemetry.update();
            timePre = timeCurrent;
        }
    }
    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        robot = new Robot(this, timer);
        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setTargetPosition(0);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTilt.setTargetPosition(0);
        robot.armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;
//        robot.init();
//        lrDrive = hardwareMap.dcMotor.get("lrDrive");
//        lfDrive = hardwareMap.dcMotor.get("lfDrive");
//        rrDrive = hardwareMap.dcMotor.get("rrDrive");
//        rfDrive = hardwareMap.dcMotor.get("rfDrive");

        //Set directions
//        lrDrive.setDirection(DcMotor.Direction.REVERSE);
//        lfDrive.setDirection(DcMotor.Direction.REVERSE);
//        rrDrive.setDirection(DcMotor.Direction.FORWARD);
//        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }

    private double calcWinchPower(double leftStickY2, double maxPower){
        double power;
        if(leftStickY2 > maxPower){
            power = maxPower;
        }
        else if(leftStickY2 < -maxPower){
            power = -maxPower;
        }
        else
        {
            power = Math.round(leftStickY2 * 100.0) / 100.0;
        }
        return power;
    }
}