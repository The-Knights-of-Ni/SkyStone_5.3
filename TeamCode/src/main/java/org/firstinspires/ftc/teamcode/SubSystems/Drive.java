
package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;

/** Mecanum drivetrain subsystem */
public class Drive extends Subsystem {
    //DC Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    //Sensors
    public BNO055IMU imu;

    //DO WITH ENCODERS
    private static final double     COUNTS_PER_MOTOR_REV_20         = 537.6*0.646;    // AM Orbital 20 motor
    private static final double     DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES           = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH                 = (COUNTS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     COUNTS_CORRECTION_X             = 0.939;
    private static final double     COUNTS_CORRECTION_Y             = 0.646;

    private static final double     WINCH_DIAMETER_INCH                 = 1.244;  //inch original measurement
    private static final double     WINCH_DIAMETER_MM                   = WINCH_DIAMETER_INCH * 2.54 * 10.0; //milimeters
    private static final double     WINCH_RADIUS_MM                     = WINCH_DIAMETER_MM / 2.0;
    private static final double     WINCH_CIRCUMFERENCE_MM              = WINCH_RADIUS_MM * 2.0 * Math.PI;
    private static final double     MOTOR_TICK_PER_REV_NEVERREST40      = 1120.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET223   = 188.3;
    private static final double     REV_PER_MIN_YELLOJACKET223          = 223.0;
    private static final double     WINCH_MAX_SPEED_MM_PER_SEC          = (160.0 * WINCH_DIAMETER_MM * Math.PI) / 60.0;
    private static final double     WINCH_MAX_SPEED_TICK_PER_SEC        = (MOTOR_TICK_PER_REV_NEVERREST40 * 160.0) / 60.0;
    private static final double     TILT_MAX_SPEED_TICK_PER_SEC         = (MOTOR_TICK_PER_REV_YELLOJACKET223 * REV_PER_MIN_YELLOJACKET223) / 60.0;



    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight, BNO055IMU imu, ElapsedTime timer) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getWinchMaxSpeedMMpSec(){
        return WINCH_MAX_SPEED_MM_PER_SEC;
    }

    public double getWinchMaxSpeedTickPerSec(){
        return WINCH_MAX_SPEED_TICK_PER_SEC;
    }

    public double getTiltMaxSpeedTickPerSec(){
        return TILT_MAX_SPEED_TICK_PER_SEC;
    }


    /**
     * Stops all drive motors
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Sets all drive motors to specified run mode
     */
    public void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontLeft.setZeroPowerBehavior(mode);
        frontRight.setZeroPowerBehavior(mode);
        rearLeft.setZeroPowerBehavior(mode);
        rearRight.setZeroPowerBehavior(mode);
    }

    public void turnRobot(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        rearLeft.setPower(power);
        rearRight.setPower(-power);
    }

    public void turn(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        rearLeft.setPower(power);
        rearRight.setPower(-power);
    }


    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        rearLeft.setTargetPosition(targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    /**
     * Positive encoder values correspond to rightward robot movement
     */
    public void strafe(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(-targetPosition);
        rearLeft.setTargetPosition(-targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    public double getYaw() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void turnByAngle(double power, double turnAngle) {
        double initialAngle = getYaw();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (turnAngle > 0.0) {
            // counter-clockwise
            double currentAngle = initialAngle;
            while (Math.abs(currentAngle - initialAngle - turnAngle) > 2) {
                turn(-power);
                currentAngle = getYaw();
                if (currentAngle < initialAngle) {
                    // angle wraparound
                    currentAngle += 360.0;
                }
            }
        } else {
            // clockwise
            double currentAngle = initialAngle;
            while (Math.abs(currentAngle - initialAngle - turnAngle) > 2) {
                turn(power);
                currentAngle = getYaw();
                if (currentAngle > initialAngle) {
                    // angle wraparound
                    currentAngle -= 360.0;
                }
            }
        }
        stop();
    }

    public void moveToPos2D(double power, double targetPositionX, double targetPositionY){
        // move to X, Y position relative to the robot coordinate system
        // the center of robot is 0,0
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        // convert from inches to motor counts
        // correct for X and Y motion asymmetry
        double distanceCountX, distanceCountY;
        distanceCountX = targetPositionX * COUNTS_PER_INCH * COUNTS_CORRECTION_X;
        distanceCountY = targetPositionY * COUNTS_PER_INCH * COUNTS_CORRECTION_Y;
        setTargetPosition2D(distanceCountX, distanceCountY);
        setPower2D(distanceCountX, distanceCountY, power);
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {

        }
        stop();
    }

    private void setPower2D(double targetPositionX, double targetPositionY, double motorPower) {
        // distribute power appropriately according to the direction of motion
        double[] motorPowers = calcMotorPowers2D(targetPositionX, targetPositionY, motorPower);
        rearLeft.setPower(motorPowers[0]);
        frontLeft.setPower(motorPowers[1]);
        rearRight.setPower(motorPowers[2]);
        frontRight.setPower(motorPowers[3]);
    }

    private void setTargetPosition2D(double targetPositionX, double targetPositionY) {
        // set motor rotation targets appropriately according to the direction of motion
        frontLeft.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        frontRight.setTargetPosition((int) ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        rearLeft.setTargetPosition((int)   ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        rearRight.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
    }

    private double[] calcMotorPowers2D(double targetPositionX, double targetPositionY, double motorPower)
    {
        // targetPositionX and targetPositionY determine the direction of movement
        // motorPower determines the magnitude of motor power
        double angleScale = Math.abs(targetPositionX) + Math.abs(targetPositionY);
        double lrPower = motorPower * (- targetPositionX + targetPositionY) / angleScale;
        double lfPower = motorPower * (+ targetPositionX + targetPositionY) / angleScale;
        double rrPower = motorPower * (+ targetPositionX + targetPositionY) / angleScale;
        double rfPower = motorPower * (- targetPositionX + targetPositionY) / angleScale;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }
}