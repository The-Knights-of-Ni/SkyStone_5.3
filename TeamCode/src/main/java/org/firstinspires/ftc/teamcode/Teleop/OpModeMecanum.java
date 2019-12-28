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

@TeleOp(name = "Mecanum")
public class OpModeMecanum extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();
        while(opModeIsActive()) {
            //Get gamepad inputs
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            boolean aButton = gamepad1.a;
            boolean bButton = gamepad1.b;
            boolean dPadUp = gamepad1.dpad_up;
            boolean dPadDown = gamepad1.dpad_down;
            boolean dPadLeft = gamepad1.dpad_left;
            boolean dPadRight = gamepad1.dpad_right;

            double leftStickX2 = gamepad2.left_stick_x;
            double leftStickY2 = -gamepad2.left_stick_y;
            double rightStickX2 = gamepad2.right_stick_x;
            boolean aButton2 = gamepad2.a;
            boolean bButton2 = gamepad2.b;
            boolean dPadUp2 = gamepad2.dpad_up;
            boolean dPadDown2 = gamepad2.dpad_down;
            boolean dPadLeft2 = gamepad2.dpad_left;
            boolean dPadRight2 = gamepad2.dpad_right;

            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
            robot.rearLeftDriveMotor.setPower(-motorPowers[0]);
            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
            robot.rearRightDriveMotor.setPower(motorPowers[2]);
            robot.frontRightDriveMotor.setPower(motorPowers[3]);

            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Left Stick Y", -leftStickY);
            telemetry.addData("Right Stick X", rightStickX);

            telemetry.addData("", "");
            telemetry.addData("Left Rear Power", robot.rearLeftDriveMotor.getPower());
            telemetry.addData("Left Front Power", robot.frontLeftDriveMotor.getPower());
            telemetry.addData("Right Rear Power", robot.rearRightDriveMotor.getPower());
            telemetry.addData("Right Front Power", robot.frontRightDriveMotor.getPower());
            telemetry.update();
        }
    }
    private void initOpMode() {
        //Initialize DC motor objects
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer);
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
}