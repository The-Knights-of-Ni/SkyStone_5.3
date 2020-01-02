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

@TeleOp(name="OmniDirectionalDrive")
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

        // wait for 1 second
        sleep(1000);


        while (opModeIsActive())
        {
            //Get gamepad inputs
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;


            //Find the angle of the robot and convert it out of euler angle form
            robotAngle = imu.getAngularOrientation().firstAngle;
            robotAngle360 = to360(robotAngle);


            //Determines the angle of the joystick and converts it out of euler angle form
            goalAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_x,gamepad1.left_stick_y));
            goalAngle360 = to360(goalAngle);


            //Find the speed for the robot to go at.
            double magnitude = Math.hypot(leftStickX, leftStickY);

            //Determines if the robot turns left or right and at what speed
            double rotationDirection = findRotationDirection(robotAngle360, goalAngle360);
            double rotationSpeed = 0.3;

            //Decides whether to stop the robot to turn or turn while the robot is moving
            if (smallestAngleBetween(robotAngle360, goalAngle360) < 20) {
                drive(magnitude,rotationSpeed*rotationDirection,rightStickX);
            } else {
                stopMotors();
                drive(0,rotationSpeed*rotationDirection,rightStickX);
            }


            telemetry.addData("Robot Angle", robotAngle);
            telemetry.update();

            resetAngle();
        }



        // turn the motors off.
        stopMotors();
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

    private void drive(double speed, double rotate, double rightStickX) {
        //Calculates and applies the powers for the motors
        //Accepts inputs of (0-1,0-1, gamepad input)
        robot.drive.frontLeft.setPower(speed + rightStickX + rotate);
        robot.drive.rearLeft.setPower(speed - rightStickX + rotate);
        robot.drive.frontRight.setPower(speed - rightStickX - rotate);
        robot.drive.rearRight.setPower(speed + rightStickX - rotate);
    }

    private void stopMotors() {
        //Stops the motor
        robot.frontLeftDriveMotor.setPower(0);
        robot.frontRightDriveMotor.setPower(0);
        robot.rearLeftDriveMotor.setPower(0);
        robot.rearRightDriveMotor.setPower(0);
    }

    private double to360(double angle) {
        //Converts from euler units to 360 degrees
        //Goes from 0 to 360 in a counter-clockwise fasion
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
        double distanceBetween = angle2 - angle1;
        if ((360 - distanceBetween) < distanceBetween) {
            return 360 - distanceBetween;
        } else {
            return distanceBetween;
        }
    }

    private double findRotationDirection(double robot, double goal) {
        //Determines the shortest way to rotate to goal angle
        if (robot <= 180) {
            if (goal < robot || goal > robot + 180) {
                return -1;
            } else {
                return 1;
            }
        } else {
            if (goal > robot || goal < robot - 180) {
                return 1;
            } else {
                return -1;
            }
        }
    }
}
