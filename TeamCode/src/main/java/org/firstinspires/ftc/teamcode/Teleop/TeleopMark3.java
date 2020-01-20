package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

/**
 * Complete Teleop Program
 * Created by AndrewC on 1/18/2020
 */
@TeleOp(name = "TeleopMark3")
public class TeleopMark3 extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    double mainArmHorizontalPos = 0.0;
    double mainArmVerticalPos = 0.0;
    double mainArmHorizontalMax = 1000.0;
    double mainArmVerticalMax = 1200.0;
    double mainArmIncrement = 500.0;
    double mainClawRotationAngle;
    double mainClawRotationIncrement = 200;
    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;

    private boolean mainClawArmControlDigital = true;
    private boolean mainClawArmDeployed = false;
    private boolean csClawArmControlDigital = true;
    private boolean csClawArmDeployed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        // call initServosTeleop() after running Auto program
//        robot.initServosTeleop();
        // call initServosAuto() if testing Teleop stand-alone
        robot.initServosAuto();
        waitForStart();

        mainClawRotationAngle = robot.control.getMainClawRotationDegrees();
        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;
        while(opModeIsActive()) {

            // Get gamepad inputs
            robot.getGamePadInputs();

            // Get the current time
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // Drive the motors
            double[] motorPowers = robot.drive.calcMotorPowers(robot.leftStickX, robot.leftStickY, robot.rightStickX);
            robot.drive.setDrivePowers(motorPowers);

            // move foundation claw
            if (robot.bumperLeft && (!robot.bumperRight)) { // foundation claw up
                robot.control.raiseClawsFromFoundation();
            } else if (robot.bumperRight && (!robot.bumperLeft)) { // foundation claw down
                robot.control.lowerClawsToFoundation();
            }
            if ((robot.triggerLeft > 0.5) && (robot.triggerRight < 0.5)) { // foundation claw up
                robot.control.raiseClawsFromFoundation();
            } else if ((robot.triggerRight > 0.5) && (robot.triggerLeft < 0.5)) { // foundation claw down
                robot.control.lowerClawsToFoundation();
            }

            // deploy main claw arm
            if (mainClawArmControlDigital) {
                if (robot.bumperRight2 && !robot.isrBumper2PressedPrev) { // toggle main claw arm deploy mode
                    if (mainClawArmDeployed) {
                        robot.control.retractMainClawArm();
                        mainClawArmDeployed = false;
                    }
                    else {
                        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
                        mainClawArmDeployed = true;
                    }
                }
            }

            // deploy capstone claw arm
            if (csClawArmControlDigital) {
                if (robot.bumperLeft2 && !robot.islBumper2PressedPrev) { // toggle capstone claw arm deploy mode
                    if (csClawArmDeployed) {
                        robot.control.retractCSClawArm();
                        csClawArmDeployed = false;
                    }
                    else {
                        robot.control.setCSClawArmDegrees(robot.control.getMainArmTargetAngle());
                        csClawArmDeployed = true;
                    }
                }
            }

            // control main claw
            if ((robot.triggerLeft2 > 0.5) && (robot.triggerRight2 < 0.5)) { // main claw open
                robot.control.openMainClaw();
            } else if ((robot.triggerRight2 > 0.5) && (robot.triggerLeft2 < 0.5)) { // main claw close
                robot.control.closeMainClawStone();
            }

            // control capstone claw
            if (robot.yButton2 && (!robot.aButton2)) { // capstone claw open
                robot.control.openCSClaw();
            } else if (robot.aButton2 && (!robot.yButton2)) { // capstone claw close
                robot.control.closeCSClaw();
            }

            // move robot main arm
            // move robot main arm along horizontal line
            if(robot.leftStickY2 >= 0.1){
                mainArmHorizontalPos = mainArmHorizontalPos + (robot.leftStickY2 - 0.1) * mainArmIncrement * deltaT/1e9;
            }
            else if(robot.leftStickY2  <= -0.1){
                mainArmHorizontalPos = mainArmHorizontalPos + (robot.leftStickY2 + 0.1) * mainArmIncrement * deltaT/1e9;
            }
            if (mainArmHorizontalPos > mainArmHorizontalMax) {
                mainArmHorizontalPos = mainArmHorizontalMax;
            }
            if (mainArmHorizontalPos < 0.0) {
                mainArmHorizontalPos =0.0;
            }
            // move robot main arm along vertical line
            if(robot.rightStickY2 >= 0.1){
                mainArmVerticalPos = mainArmVerticalPos + (robot.rightStickY2 - 0.1) * mainArmIncrement * deltaT/1e9;
            }
            else if(robot.rightStickY2  <= -0.1){
                mainArmVerticalPos = mainArmVerticalPos + (robot.rightStickY2 + 0.1) * mainArmIncrement * deltaT/1e9;
            }
            if (mainArmVerticalPos > mainArmVerticalMax) {
                mainArmVerticalPos = mainArmVerticalMax;
            }
            if (mainArmVerticalPos < 0.0) {
                mainArmVerticalPos =0.0;
            }
            robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);

            // rotate main claw
            if(robot.rightStickX2 >= 0.1){
                mainClawRotationAngle = mainClawRotationAngle + (robot.rightStickX2 - 0.1) * mainClawRotationIncrement * deltaT/1e9;
            }
            else if(robot.rightStickX2  <= -0.1){
                mainClawRotationAngle = mainClawRotationAngle + (robot.rightStickX2 + 0.1) * mainClawRotationIncrement * deltaT/1e9;
            }
            if (mainClawRotationAngle > 180.0) {
                mainClawRotationAngle = 180.0;
            }
            if (mainClawRotationAngle < 0.0) {
                mainClawRotationAngle =0.0;
            }
            robot.control.setMainClawRotationDegrees(mainClawRotationAngle);

            if(mainClawArmControlDigital && mainClawArmDeployed){
                robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
            }
            if(csClawArmDeployed && csClawArmControlDigital){
                robot.control.setCSClawArmDegrees(robot.control.getMainArmTargetAngle());
            }

            //Automate skybrige pos
            if(robot.bButton2 && !robot.isbButton2PressedPrev){
                robot.control.setMainArmPosition(80.0, 50.0);
            }

            telemetry.addData("X", mainArmHorizontalPos);
            telemetry.addData("Y", mainArmVerticalPos);
            telemetry.addData("clawRotation", mainClawRotationAngle);
            telemetry.addData("rightStickX2", robot.rightStickX2);
//            telemetry.addData("Right Rear Power", robot.rearRightDriveMotor.getPower());
//            telemetry.addData("Right Front Power", robot.frontRightDriveMotor.getPower());

            telemetry.update();
        }
    }

    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        robot = new Robot(this, timer);

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

}