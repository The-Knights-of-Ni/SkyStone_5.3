package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

/**
 * Created by tarunsingh on 12/5/17.
 * Modified by AndrewC on 1/17/2020.
 */

@TeleOp(name="Servo Test")
public class ServoTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private static final float mmPerInch        = 25.4f;

    private Robot robot;

    double timePre;
    double timeCurrent;
    ElapsedTime timer;

    public void initOpMode() throws IOException {
        timer = new ElapsedTime();
        this.robot = new Robot(this, timer);

    }
    public void runOpMode() {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        robot.initServosAuto();
        telemetry.clearAll();
//        telemetry.addLine("Wait For Start");
//        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            //Get gamepad inputs
            robot.getGamePadInputs();

            //Get the current time
            timeCurrent = timer.nanoseconds();

//            if (aButton && !isaButtonPressedPrev && ((robot.mainClaw.getPosition()+0.05) < 1.0)) {
//                robot.mainClaw.setPosition(robot.mainClaw.getPosition() + 0.05);
//                isaButtonPressedPrev = true;
//            }
//            if(bButton && !isbButtonPressedPrev && ((robot.mainClaw.getPosition()-0.05) > 0.0)){
//                robot.mainClaw.setPosition(robot.mainClaw.getPosition() - 0.05);
//                isbButtonPressedPrev = true;
//            }
//            if (!aButton) {
//                isaButtonPressedPrev = false;
//            }
//            if(!bButton){
//                isbButtonPressedPrev = false;
//            }
//
//            if (xButton && !isxButtonPressedPrev) {
//                robot.mainClaw.setPosition(0.88);
//                isxButtonPressedPrev = true;
//            }
//            if(yButton && !isyButtonPressedPrev){
//                robot.mainClaw.setPosition(0.13);
//                isyButtonPressedPrev = true;
//            }
//            if (!xButton) {
//                isxButtonPressedPrev = false;
//            }
//            if(!yButton){
//                isyButtonPressedPrev = false;
//            }

            // test main arm servos

            if (robot.leftStickX > 0.5) {
                robot.control.modifyServo(robot.mainClawArm,0.005);
            }
            else if (robot.leftStickX > 0.1) {
                robot.control.modifyServo(robot.mainClawArm,0.001);
            }
            else if (robot.leftStickX > -0.1) {
                robot.control.modifyServo(robot.mainClawArm,0.0);
            }
            else if (robot.leftStickX > -0.5) {
                robot.control.modifyServo(robot.mainClawArm,-0.001);
            }
            else {
                robot.control.modifyServo(robot.mainClawArm,-0.005);
            }

            if (robot.rightStickY > 0.5) {
                robot.control.modifyServo(robot.mainClawRotation,0.005);
            }
            else if (robot.rightStickY > 0.1) {
                robot.control.modifyServo(robot.mainClawRotation,0.001);
            }
            else if (robot.rightStickY > -0.1) {
                robot.control.modifyServo(robot.mainClawRotation,0.0);
            }
            else if (robot.rightStickY > -0.5) {
                robot.control.modifyServo(robot.mainClawRotation,-0.001);
            }
            else {
                robot.control.modifyServo(robot.mainClawRotation,-0.005);
            }

            if (robot.rightStickX > 0.5) {
                robot.control.modifyServo(robot.mainClaw,0.005);
            }
            else if (robot.rightStickX > 0.1) {
                robot.control.modifyServo(robot.mainClaw,0.001);
            }
            else if (robot.rightStickX > -0.1) {
                robot.control.modifyServo(robot.mainClaw,0.0);
            }
            else if (robot.rightStickX > -0.5) {
                robot.control.modifyServo(robot.mainClaw,-0.001);
            }
            else {
                robot.control.modifyServo(robot.mainClaw,-0.005);
            }

            telemetry.addData("Main Arm ", "%.3f", robot.mainClawArm.getPosition());
            telemetry.addData("Main Claw Rotation ", "%.3f", robot.mainClawRotation.getPosition());
            telemetry.addData("Main Claw ", "%.3f", robot.mainClaw.getPosition());

            // test capstone arm servos

            if (robot.leftStickX2 > 0.5) {
                robot.control.modifyServo(robot.csArm,0.005);
            }
            else if (robot.leftStickX2 > 0.1) {
                robot.control.modifyServo(robot.csArm,0.001);
            }
            else if (robot.leftStickX2 > -0.1) {
                robot.control.modifyServo(robot.csArm,0.0);
            }
            else if (robot.leftStickX2 > -0.5) {
                robot.control.modifyServo(robot.csArm,-0.001);
            }
            else {
                robot.control.modifyServo(robot.csArm,-0.005);
            }

            if (robot.rightStickX2 > 0.5) {
                robot.control.modifyServo(robot.csClaw,0.005);
            }
            else if (robot.rightStickX2 > 0.1) {
                robot.control.modifyServo(robot.csClaw,0.001);
            }
            else if (robot.rightStickX2 > -0.1) {
                robot.control.modifyServo(robot.csClaw,0.0);
            }
            else if (robot.rightStickX2 > -0.5) {
                robot.control.modifyServo(robot.csClaw,-0.001);
            }
            else {
                robot.control.modifyServo(robot.csClaw,-0.005);
            }

            telemetry.addData("Capstone Arm ", "%.3f", robot.csArm.getPosition());
            telemetry.addData("Capstone Claw ", "%.3f", robot.csClaw.getPosition());

            // test foundation claw servos

            if (robot.leftStickY2 > 0.5) {
                robot.control.modifyServo(robot.fClawL,0.005);
            }
            else if (robot.leftStickY2 > 0.1) {
                robot.control.modifyServo(robot.fClawL,0.001);
            }
            else if (robot.leftStickY2 > -0.1) {
                robot.control.modifyServo(robot.fClawL,0.0);
            }
            else if (robot.leftStickY2 > -0.5) {
                robot.control.modifyServo(robot.fClawL,-0.001);
            }
            else {
                robot.control.modifyServo(robot.fClawL,-0.005);
            }

            if (robot.rightStickY2 > 0.5) {
                robot.control.modifyServo(robot.fClawR,0.005);
            }
            else if (robot.rightStickY2 > 0.1) {
                robot.control.modifyServo(robot.fClawR,0.001);
            }
            else if (robot.rightStickY2 > -0.1) {
                robot.control.modifyServo(robot.fClawR,0.0);
            }
            else if (robot.rightStickY2 > -0.5) {
                robot.control.modifyServo(robot.fClawR,-0.001);
            }
            else {
                robot.control.modifyServo(robot.fClawR,-0.005);
            }

            telemetry.addData("Foundation Claw L ", "%.3f", robot.fClawL.getPosition());
            telemetry.addData("Foundation Claw R ", "%.3f", robot.fClawR.getPosition());

            telemetry.update();
            sleep(100);
        }

    }
}