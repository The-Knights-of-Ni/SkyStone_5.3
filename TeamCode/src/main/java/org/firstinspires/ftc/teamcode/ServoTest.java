package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

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

    public void initOpMode(){
        timer = new ElapsedTime();
        this.robot = new Robot(this, timer);

    }
    public void runOpMode() {
        initOpMode();
        robot.initServosAuto();
        telemetry.clearAll();
        telemetry.addLine("Wait For Start");
        telemetry.update();
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

//            // test front claw servos
//
//            robot.fClawL.setPosition(this.gamepad2.left_stick_x*0.5+0.5);
//            robot.fClawR.setPosition(this.gamepad2.right_stick_x*0.5+0.5);
//
//            telemetry.addData("LEFT Foundation Claw Servo Position", robot.fClawL.getPosition());
//            telemetry.addData("Status", "Running");
//
//            telemetry.addData("RIGHT Foundation Claw Servo Position", robot.fClawR.getPosition());
//            telemetry.addData("Status", "Running");

//            // test main arm servos
//
//            robot.mainClawArm.setPosition(this.gamepad2.left_stick_x*0.5+0.5);
//            robot.mainClawRotation.setPosition(this.gamepad2.right_stick_x*0.5+0.5);
//            robot.mainClaw.setPosition(this.gamepad2.right_stick_y*0.5+0.5);
//
//            telemetry.addData("Main Arm Servo Position", robot.mainClawArm.getPosition());
//            telemetry.addData("Main Claw Rotation Servo Position", robot.mainClawRotation.getPosition());
//            telemetry.addData("Main Claw Servo Position", robot.mainClaw.getPosition());
//            telemetry.addData("Status", "Running");

            // test capstone arm servos

            if (robot.leftStickX2 > 0.75) {
                robot.control.modifyServo(robot.csArm,0.005);
            }
            else if (robot.leftStickX2 > 0.55) {
                robot.control.modifyServo(robot.csArm,0.001);
            }
            else if (robot.leftStickX2 > 0.45) {
                robot.control.modifyServo(robot.csArm,0.0);
            }
            else if (robot.leftStickX2 > 0.25) {
                robot.control.modifyServo(robot.csArm,-0.001);
            }
            else {
                robot.control.modifyServo(robot.csArm,-0.005);
            }
            if (robot.rightStickX2 > 0.75) {
                robot.control.modifyServo(robot.csClaw,0.005);
            }
            else if (robot.rightStickX2 > 0.55) {
                robot.control.modifyServo(robot.csClaw,0.001);
            }
            else if (robot.rightStickX2 > 0.45) {
                robot.control.modifyServo(robot.csClaw,0.0);
            }
            else if (robot.rightStickX2 > 0.25) {
                robot.control.modifyServo(robot.csClaw,-0.001);
            }
            else {
                robot.control.modifyServo(robot.csClaw,-0.005);
            }

            telemetry.addData("Capstone Arm Servo Position", robot.csArm.getPosition());
            telemetry.addData("Capstone Claw Servo Position", robot.csClaw.getPosition());


            telemetry.addData("mainClaw", robot.mainClaw.getPosition());
            telemetry.update();
            sleep(10);
        }

    }
}