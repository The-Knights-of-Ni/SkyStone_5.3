package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "Strafe Program")
public class SimpleTestDrive extends LinearOpMode {
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


    double timePre;
    double timeCurrent;
    double deltaT;
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

            double[] motorPowers = calcMotorPowers(leftStickX);
            robot.rearLeftDriveMotor.setPower(motorPowers[0]);
            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
            robot.rearRightDriveMotor.setPower(motorPowers[2]);
            robot.frontRightDriveMotor.setPower(motorPowers[3]);

            deltaT = timeCurrent - timePre;


            telemetry.addData("Left Stick Y2", leftStickY2);
            telemetry.addData("Right Stick Y2", rightStickY2);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("deltaT", deltaT);



            telemetry.update();
            timePre = timeCurrent;
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

    private double[] calcMotorPowers(double leftStickX) {
        double lrPower = -leftStickX;
        double lfPower = leftStickX;
        double rrPower = leftStickX;
        double rfPower = -leftStickX;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }


}
