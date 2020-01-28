package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;

/**
 * Created by Elijah Rowe
 */

@TeleOp(name="OmniDriveTest", group = "Drive Tests")
public class OmniDriveTest extends LinearOpMode {
    private Robot robot;
    private BNO055IMU imu;
    double robotAngle;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30;

    private void initOpMode() throws IOException {
        //Initialize DC motor objects
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer);

    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        double goalAngle;
        double robotAngle360;
        double goalAngle360;


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);


        while (opModeIsActive())
        {
//            //Get gamepad inputs
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
//            double rightStickX = gamepad1.right_stick_x;
//            boolean aButton = gamepad1.a;
//            boolean bButton = gamepad1.b;
//            boolean dPadUp = gamepad1.dpad_up;
//            boolean dPadDown = gamepad1.dpad_down;
//            boolean dPadLeft = gamepad1.dpad_left;
//            boolean dPadRight = gamepad1.dpad_right;
//
//            double leftStickX2 = gamepad2.left_stick_x;
//            double leftStickY2 = -gamepad2.left_stick_y;
//            double rightStickX2 = gamepad2.right_stick_x;
//            double rightStickY2 = gamepad2.right_stick_y;
//            boolean aButton2 = gamepad2.a;
//            boolean bButton2 = gamepad2.b;
//            boolean dPadUp2 = gamepad2.dpad_up;
//            boolean dPadDown2 = gamepad2.dpad_down;
//            boolean dPadLeft2 = gamepad2.dpad_left;
//            boolean dPadRight2 = gamepad2.dpad_right;
//            boolean bumperLeft2 = gamepad2.left_bumper;
//            boolean bumperRight2 = gamepad2.right_bumper;


            goalAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x) + Math.PI / 2);
            robotAngle360 = to360(robotAngle);
            goalAngle360 = to360(goalAngle);

            double speed = smallestAngleBetween(robotAngle360, goalAngle360)/180;

            robotAngle = imu.getAngularOrientation().firstAngle;

            if (robotAngle360 <= 180) {
                if (goalAngle360 < robotAngle360 || goalAngle360 > robotAngle360 + 180) {
                    rotate(-speed);
                } else {
                    rotate(speed);
                }
            } else {
                if (goalAngle360 > robotAngle360 || goalAngle360 < robotAngle360 - 180) {
                    rotate(speed);
                } else {
                    rotate(-speed);
                }
            }
//            if (robotAngle > 0){
//                if (robotAngle < 7){
//                    stopMotor();
//                    telemetry.addData("Stopped", robotAngle);
//                    telemetry.update();
//                } else {
//                    rotate(0.3);
//                }
//
//            } else {
//                if (robotAngle > -7){
//                    stopMotor();
//                    telemetry.addData("Stopped", robotAngle);
//                    telemetry.update();
//                } else {
//                    rotate(-0.3);
//                }
//
//            }
            telemetry.addData("Robot Angle", robotAngle360);
            telemetry.addData("Goal Angle",  goalAngle);
            telemetry.addData("Goal Angle 360",  goalAngle360);
            telemetry.addData("Angle Between",  smallestAngleBetween(goalAngle360,robotAngle360));
            telemetry.addData("Speed", speed);
            telemetry.update();

            resetAngle();
        }



        // turn the motors off.
        robot.frontLeftDriveMotor.setPower(0);
        robot.frontRightDriveMotor.setPower(0);
        robot.rearLeftDriveMotor.setPower(0);
        robot.rearRightDriveMotor.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void rotate(double speed) {
        //Turns the robot to the right
        //Speed Range is 0 to 1
        robot.drive.frontLeft.setPower(speed);
        robot.drive.rearLeft.setPower(speed);
        robot.drive.frontRight.setPower(-speed);
        robot.drive.rearRight.setPower(-speed);
    }

    private void stopMotor() {
        //Stops the motor
        robot.frontLeftDriveMotor.setPower(0);
        robot.frontRightDriveMotor.setPower(0);
        robot.rearLeftDriveMotor.setPower(0);
        robot.rearRightDriveMotor.setPower(0);
    }

    private double to360(double angle) {
        //Converts from euler units to 360 degrees
        //Goes from 0 to 360 in a clockwise fasion
        //Accepts numbers between -180 and 180
        if (angle >= 0) {
            return angle;
        } else {
            return angle + 360;
        }
    }

    private double smallestAngleBetween(double angle1, double angle2) {
        //Returns the smallest angle between angle1 and angle 2
        //Accepts the range 0 - 360 for both angles
        double distanceBetween = Math.abs(angle2 - angle1);
        if ((360 - distanceBetween) < distanceBetween) {
            return 360 - distanceBetween;
        } else {
            return distanceBetween;
        }
    }
}
