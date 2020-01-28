package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

/**
 * Created by AndrewChiang on 12/27/19.
 */

//Message from Elijah, please comment your code

@TeleOp(name = "TeleopMark2")
@Disabled
public class TeleopMark2 extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    int winchCurrentPosition = 0;
    int winchTargetPositionCurrent = 0;
    int winchTargetPositionPre = 0;
    int winchPosError = 400;
    int winchMax = 8410;

    int tiltCurrentPosition = 0;
    int tiltTargetPositionCurrent = 0;
    int tiltTargetPositionPre = 0;
    int tiltPosError = 300;
    int tiltMax = 6000;
    double tiltCurrentAngle = 0;

//    int clawTiltCurrentPosition = 0;
//    int clawTiltTargetPositionCurrent = 0;
//    int clawTiltTargetPositionPre = 0;
//    int clawTiltPosError = 300;
//    int clawTiltMax = 1200;

    double winchSpeed;
    double tiltSpeed;
    double deltaT;
    double winchIncrement = 0;
    double tiltIncrement = 0;
    double timePre;
    double timeCurrent;
    ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        robot.initServosAuto();
        waitForStart();

        double tgtPower = 0;

        telemetry.clearAll();
        while(opModeIsActive()) {

            //Get gamepad inputs
            robot.getGamePadInputs();

            //Get the current time
            timeCurrent = timer.nanoseconds();

//            //Drive the motors
//            double[] motorPowers = calcMotorPowers2(leftStickX, leftStickY, rightStickX);
//            robot.rearLeftDriveMotor.setPower(motorPowers[0]);
//            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
//            robot.rearRightDriveMotor.setPower(motorPowers[2]);
//                              robot.frontRightDriveMotor.setPower(motorPowers[3]);
//
            deltaT = timeCurrent - timePre;

            //Winch
            //930mm, 8400 encoder count
            if((robot.leftStickY2 > 0.5)){
                robot.xRailWinch.setPower(1.0);
            }
            else if(robot.leftStickY2 > 0.1){
                robot.xRailWinch.setPower(0.5);
            }
            else if(robot.leftStickY2 > -0.5){
                robot.xRailWinch.setPower(0.3);
            }
            else {
                robot.xRailWinch.setPower(0.7);
            }

            if(robot.leftStickY2 >= 0.1){
                winchSpeed = (robot.leftStickY2 - 0.1) * (robot.control.getWinchMaxSpeedTickPerSec() / 0.9);
            }
            else if(robot.leftStickY2  <= -0.1){
                winchSpeed = (robot.leftStickY2 + 0.1) * (robot.control.getWinchMaxSpeedTickPerSec() / 0.9);
            }
            else{
                winchSpeed = 0.0;
            }

            //Get winch's current position
            winchCurrentPosition = robot.xRailWinch.getCurrentPosition();

            //Determine the speed of the winch
            winchIncrement = (winchSpeed * deltaT) / Math.pow(10.0,9);

            //Determine winch target position
            winchTargetPositionCurrent = (int) (winchTargetPositionPre + winchIncrement);

            //Make sure the winch is still in the boundries
            if((winchTargetPositionCurrent <= winchMax) && (winchTargetPositionCurrent >= 0)
                    && ( Math.abs(winchTargetPositionCurrent - winchCurrentPosition) < winchPosError)){
                robot.xRailWinch.setTargetPosition(winchTargetPositionCurrent);
                winchTargetPositionPre = winchTargetPositionCurrent;
            }

            //Tilt
            robot.armTilt.setPower(1.0);
//            if((robot.rightStickY2 > 0.5)){
//                robot.armTilt.setPower(1.0);
//            }
//            else if(robot.rightStickY2 > 0.1){
//                robot.armTilt.setPower(0.7);
//            }
//            else if(robot.rightStickY2 > -0.5){
//                robot.armTilt.setPower(1.0);
//            }
//            else{
//                robot.armTilt.setPower(1.0);
//            }

            if(robot.rightStickY2 >= 0.1){
                tiltSpeed = (robot.rightStickY2 - 0.1) * (robot.control.getTiltMaxSpeedTickPerSec() / 0.9);
            }
            else if(robot.rightStickY2  <= -0.1){
                tiltSpeed = (robot.rightStickY2 + 0.1) * (robot.control.getTiltMaxSpeedTickPerSec() / 0.9);
            }
            else{
                tiltSpeed = 0.0;
            }

            //Find how much the arm is tilted
            tiltCurrentPosition = robot.armTilt.getCurrentPosition();

            //Determines the speed of the tilt
            tiltIncrement = (tiltSpeed * deltaT) / Math.pow(10.0,9);

            //Determines the target tilt position
            tiltTargetPositionCurrent = (int) (tiltTargetPositionPre + tiltIncrement);

            //Make sure the tilt is tilt in bounds
            if((tiltTargetPositionCurrent <= tiltMax) && (tiltTargetPositionCurrent >= 0)
                    && ( Math.abs(tiltTargetPositionCurrent - tiltCurrentPosition) < tiltPosError)){
                robot.armTilt.setTargetPosition(tiltTargetPositionCurrent);
                tiltTargetPositionPre = tiltTargetPositionCurrent;
            }

//            tiltCurrentAngle = ((tiltCurrentPosition / 2510.0) * 90.0);
//
//            robot.mainClawArm.setPosition(robot.control.mainClawArmAngleToPos(tiltCurrentAngle));

//            if (aButton2 && !isaButton2PressedPrev) {
//                robot.mainClaw.setPosition(robot.drive.getMainClawPosClosedStone());
//                isaButton2PressedPrev = true;
//            }
//            if(bButton2 && !isbButton2PressedPrev){
//                robot.mainClaw.setPosition(robot.drive.getMainClawPosOpen());
//                isbButton2PressedPrev = true;
//            }
//            if (!aButton2) {
//                isaButton2PressedPrev = false;
//            }
//            if(!bButton2){
//                isbButton2PressedPrev = false;
//            }

//            //Find how much the claw is tilted
//            clawTiltCurrentPosition = (int) robot.mainArm.getPosition();
//
//            //Determines the speed of the winch
//            tiltIncrement = (tiltSpeed * deltaT) / Math.pow(10.0,9);
//
//            //Determines the target tilt position
//            tiltTargetPositionCurrent = (int) (tiltTargetPositionPre + tiltIncrement);
//
//            //Make sure the tilt is tilt in bounds
//            if((tiltTargetPositionCurrent <= tiltMax) && (tiltTargetPositionCurrent >= 0)
//                    && ( Math.abs(tiltTargetPositionCurrent - tiltCurrentPosition) < tiltPosError)){
//                robot.armTilt.setTargetPosition(tiltTargetPositionCurrent);
//                tiltTargetPositionPre = tiltTargetPositionCurrent;
//            }

//            if(bumperLeft){
//                robot.rearLeftDriveMotor.setPower(0);
//                robot.frontLeftDriveMotor.setPower(0);
//                robot.rearRightDriveMotor.setPower(0);
//                robot.frontRightDriveMotor.setPower(0);
//            }
//
//
//            telemetry.addData("Left Stick Y2", leftStickY2);
//            telemetry.addData("Right Stick Y2", rightStickY2);
//            telemetry.addData("Right Stick X", rightStickX);


            telemetry.addData("currentPosWinch", winchCurrentPosition);
            telemetry.addData("targetPosWinch", winchTargetPositionCurrent);
            telemetry.addData("incrementWinch", winchIncrement);
//
//            telemetry.addData("mainArm", robot.mainArm.getPosition());
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
        try {
            robot = new Robot(this, timer);
        } catch (IOException e) {
            e.printStackTrace();
        }
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

    private double[] calcMotorPowers2(double leftStickX, double leftStickY, double rightStickX) {
        if(Math.abs(leftStickX) >= Math.abs((leftStickY))){
            leftStickY = 0;
        }
        else{
            leftStickX = 0;
        }
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