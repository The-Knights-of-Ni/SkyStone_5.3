package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Elijah Rowe
 */

@TeleOp(name="Omnidirectional Drive", group="Assisted Driving")
public class OmniDirectionalDrive extends LinearOpMode {
    private Robot robot;
    private BNO055IMU imu;
    double robotAngle;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30;

    private void initOpMode() {
        //Initialize DC motor objects
        ElapsedTime timer = new ElapsedTime();
        robot = new Robot(this, timer);

    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        initOpMode();

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

        double robotAngle;


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

        // wait for 1 second
        sleep(1000);


        while (opModeIsActive())
        {
            //Get gamepad inputs
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            double rightStickY = gamepad1.right_stick_y;

            //Find the goal angle from the controller
            double goalAngle = -Math.toDegrees(Math.atan2(leftStickY, leftStickX) - Math.PI / 2);
            double goalAngle360 = to360(goalAngle);

            //Find the angle of the robot and convert it out of euler angle form
            robotAngle = Math.toDegrees(imu.getAngularOrientation().firstAngle);
            double robotAngle360 = to360(robotAngle);
            double correction = smallestAngleBetween(robotAngle360,goalAngle);


//            //Drive the robot
//            double motorPowers[] = calcMotorPowers(leftStickX,leftStickY,rightStickX,correction);
//            robot.rearLeftDriveMotor.setPower(motorPowers[0]);
//            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
//            robot.rearRightDriveMotor.setPower(motorPowers[2]);
//            robot.frontRightDriveMotor.setPower(motorPowers[3]);


            telemetry.addData("Correction", correction);
            telemetry.addData("Goal angle", goalAngle);
            telemetry.addData("Goal angle 360", goalAngle360);
            telemetry.addData("Robot angle", robotAngle);
            telemetry.addData("Robot angle 360", robotAngle360);
            telemetry.update();

            resetAngle();


    }

        // turn the motors off.
        stopMotors();
    }


    private void resetAngle() {
        //Resets the global angle
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void stopMotors() {
        //Stops the motor
        robot.frontLeftDriveMotor.setPower(0);
        robot.frontRightDriveMotor.setPower(0);
        robot.rearLeftDriveMotor.setPower(0);
        robot.rearRightDriveMotor.setPower(0);
    }

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX, double correction) {
        //Calculates the powers of each motor based on the controller input
        //Accepts controller inputs from xbox joystick
        //LeftStickX - strafe, LeftStickY - forward/backwards, rightStickJoystick controls turn angle
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double rearLeftPower = r * Math.sin(robotAngle) + rightStickX;
        double frontLeftPower = r * Math.cos(robotAngle) + rightStickX;
        double rearRightPower = r * Math.cos(robotAngle) - rightStickX;
        double frontRightPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{rearLeftPower, frontLeftPower, rearRightPower, frontRightPower};
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
        //Returns the smallest angle between angle1 and angle 2 and whether angle2 is to the left or right of angle 1
        //Negative is counter-clockwise or to the left
        //Accepts the range 0 - 360 for both angles, no euler units
        double i;
        double smallestAngleBetween;
        double distanceBetween = Math.abs(angle2 - angle1);

        if (angle1 <= 180) {
            if (angle2 < angle1 || angle2 > angle1 + 180) {
                i= -1;
            } else {
                i = 1;
            }
        } else {
            if (angle2 < angle1 || angle2 > angle1 - 180) {
                i = -1;
            } else {
                i = 1;
            }
        }

        if ((360 - distanceBetween) < distanceBetween) {
            smallestAngleBetween = 360 - distanceBetween;
        } else {
            smallestAngleBetween = distanceBetween;
        }

        return smallestAngleBetween*i;
    }
}

