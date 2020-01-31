package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

/**
 * Created by tarunsingh on 12/5/17.
 */

@Autonomous(name="Encoder Test")
@Disabled
public class EncoderTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private Robot robot;
    private double currentPos;

    public void initOpMode() throws IOException {
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer);
        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setTargetPosition(0);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xRailWinch.setTargetPosition(0);
        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("OK!", 5);
    }
    public void runOpMode() {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();
        robot.timer.reset();


//
        while (opModeIsActive()) {
            currentPos = robot.armTilt.getCurrentPosition();
            telemetry.addData("tilt", currentPos);
            telemetry.addData("tiltAngle", (currentPos / 2510.0) * 90.0);

            telemetry.update();
        }
//        robot.xRailWinch.setPower(0.5);
//        robot.xRailWinch.setTargetPosition(8400);
//        while(robot.xRailWinch.isBusy()){
//            telemetry.addData("Winch", robot.xRailWinch.getCurrentPosition());
//            telemetry.update();
//        }
//        telemetry.addData("Winch", robot.xRailWinch.getCurrentPosition());
//        telemetry.update();


        sleep(10000);
    }
}