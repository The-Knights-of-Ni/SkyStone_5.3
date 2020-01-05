
package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/** Mecanum drivetrain subsystem */
public class Drive extends Subsystem {

    private Robot robot;
    //DC Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;
    private OpMode opMode;
    private HardwareMap hardwareMap;

    private double robotCurrentPosX;    // unit in mm
    private double robotCurrentPosY;    // unit in mm
    private double robotCurrentAngle;   // unit in degrees



    //Sensors
    public BNO055IMU imu;

    //DO WITH ENCODERS
    private static final double     TICKS_PER_MOTOR_REV_20          = 537.6;    // AM Orbital 20 motor
    private static final double     RPM_MAX_NEVERREST_20            = 340;
    private static final double     ANGULAR_V_MAX_NEVERREST_20      = (TICKS_PER_MOTOR_REV_20 * RPM_MAX_NEVERREST_20) / 60.0;
    private static final double     DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES           = 4.0 ;     // For figuring circumference
    private static final double     WHEEL_DIAMETER_MM               = 4.0 * 2.54 * 10.0;
    private static final double     COUNTS_PER_INCH                 = (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     COUNTS_PER_MM                 = (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
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
    private static final double     TILT_TICK_PER_90_DEGREE             = 2510.0;

    private static final double     CLAW_ARM_POS_0_DEG                  = 0.08; // xRail horizontal and main claw facing down
    private static final double     CLAW_ARM_POS_180_DEG                = 0.85;

    private static final double     DRIVE_SPEED             = 0.4;
    private static final double     TURN_SPEED              = 0.3;

    private static final double     ROBOT_INIT_POS_X    = 15.0;
    private static final double     ROBOT_INIT_POS_Y    = 15.0;
    private static final double     ROBOT_INIT_ANGLE    = 45.0;
    private static final float      mmPerInch        = 25.4f;

    // Servos
    private static final double fClawLFoundation = 0.45;
    private static final double fClawRFoundation = 0.56;
    private static final double fClawLDown = 0.36;
    private static final double fClawLUp = 0.8;
    private static final double fClawRUp = 0.11;
    private static final double fClawRDown = 0.63;

    private static final double mainArmDown = 0.5; // TEMPORARY
    private static final double mainClawOpen = 0.8; // TEMPORARY
    private static final double mainClawClosed = 0.3; // TEMPORARY
    private static final double mainClawStone = 0.5; // TEMPORARY

    private boolean allianceRed = false;


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;


    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight, BNO055IMU imu, ElapsedTime timer, OpMode opMode) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.hardwareMap = opMode.hardwareMap;
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
    public double getAngularVMaxNeverrest20(){
        return ANGULAR_V_MAX_NEVERREST_20;
    }
    public double getMotorTickPerRevYellojacket223(){
        return MOTOR_TICK_PER_REV_YELLOJACKET223;
    }
    public double getTiltTickPer90Degree(){
        return TILT_TICK_PER_90_DEGREE;
    }
    public double getClawArmPos0Deg(){
        return CLAW_ARM_POS_0_DEG;
    }
    public double getClawArmPos180Deg(){
        return CLAW_ARM_POS_180_DEG;
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
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        // convert from inches to motor counts
        // correct for X and Y motion asymmetry
        double distanceCountX, distanceCountY;
        distanceCountX = targetPositionX * COUNTS_PER_MM * COUNTS_CORRECTION_X;
        distanceCountY = targetPositionY * COUNTS_PER_MM * COUNTS_CORRECTION_Y;
        setTargetPosition2D(distanceCountX, distanceCountY);
        setPower2D(distanceCountX, distanceCountY, power);
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {

        }
        stop();
    }

    public void setPower2D(double targetPositionX, double targetPositionY, double motorPower) {
        // distribute power appropriately according to the direction of motion
        double[] motorPowers = calcMotorPowers2D(targetPositionX, targetPositionY, motorPower);
        rearLeft.setPower(motorPowers[0]);
        frontLeft.setPower(motorPowers[1]);
        rearRight.setPower(motorPowers[2]);
        frontRight.setPower(motorPowers[3]);
    }

    public void setTargetPosition2D(double targetPositionX, double targetPositionY) {
        // set motor rotation targets appropriately according to the direction of motion
        frontLeft.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        frontRight.setTargetPosition((int) ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        rearLeft.setTargetPosition((int)   ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        rearRight.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
    }

    public double[] calcMotorPowers2D(double targetPositionX, double targetPositionY, double motorPower)
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

    public void turnRobot(double degrees) {
        this.turnByAngle(TURN_SPEED, degrees);
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        opMode.telemetry.addData("turnRobot",  "turn to %7.2f degrees", robotCurrentAngle);
        opMode.telemetry.update();
//        opMode.sleep(100);
    }

    public void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double  deltaX = targetPositionX - robotCurrentPosX;    // in absolute field coordinate
        double  deltaY = targetPositionY - robotCurrentPosY;    // in absolute field coordinate
        double  distanceCountX, distanceCountY;  // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX = deltaX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0)
                + deltaY * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        distanceCountY = deltaX * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + deltaY * Math.sin(robotCurrentAngle*Math.PI/180.0);
        this.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        opMode.telemetry.addData("moveToPosABS",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        this.moveToPos2D(DRIVE_SPEED, targetPositionX, targetPositionY);
        robotCurrentPosX += targetPositionY * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += targetPositionY * Math.sin(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveToPosREL",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveForward(double distance) {
        this.moveToPos2D(DRIVE_SPEED, 0.0, distance);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveBackward(double distance) {
        this.moveToPos2D(DRIVE_SPEED, 0.0, -distance);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveLeft(double distance) {
        this.moveToPos2D(DRIVE_SPEED, -distance, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveRight(double distance) {
        this.moveToPos2D(DRIVE_SPEED, distance, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void lowerClawsToFoundation() {
        robot.fClawL.setPosition(fClawLDown);
        robot.fClawR.setPosition(fClawRDown);
    }

    public void pickUpStone() {
        // Lower arm
        // need to add rotation, arm claw synchronization
        robot.mainArm.setPosition(mainArmDown);
        robot.mainClaw.setPosition(mainClawOpen);
        robot.mainClaw.setPosition(mainClawStone);
        robot.mainArm.setPosition(0.5);
    }

    public void park() {
        moveToPos2D(0.25,15,15); //temp position
    }

    public double mainArmAngletoPos(double angle){
        return ((angle / 180.0) * (this.getClawArmPos180Deg() - this.getClawArmPos0Deg()));
    }
}