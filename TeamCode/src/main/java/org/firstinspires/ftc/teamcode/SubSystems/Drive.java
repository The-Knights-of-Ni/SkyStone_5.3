
package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/** Mecanum drivetrain subsystem */
public class Drive extends Subsystem {
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    //DC Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    //Sensors
    private BNO055IMU imu;

    private double robotCurrentPosX;    // unit in mm
    private double robotCurrentPosY;    // unit in mm
    private double robotCurrentAngle;   // unit in degrees

    private int encoderOffsetFL = 0;
    private int encoderOffsetFR = 0;
    private int encoderOffsetRL = 0;
    private int encoderOffsetRR = 0;

    //DO WITH ENCODERS
    private static final double     TICKS_PER_MOTOR_REV_20          = 537.6;    // AM Orbital 20 motor
    private static final double     RPM_MAX_NEVERREST_20            = 340;
    private static final double     ANGULAR_V_MAX_NEVERREST_20      = (TICKS_PER_MOTOR_REV_20 * RPM_MAX_NEVERREST_20) / 60.0;
    private static final double     DRIVE_GEAR_REDUCTION            = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES           = 100.0/25.4 ;     // For figuring circumference
    private static final double     WHEEL_DIAMETER_MM               = 100.0;
    private static final double     COUNTS_PER_INCH                 = (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     COUNTS_PER_MM                 = (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    private static final double     COUNTS_CORRECTION_X             = 1.08;
    private static final double     COUNTS_CORRECTION_Y             = 0.91;
    private static final double     COUNTS_PER_DEGREE             = 10.833*1.02;     // 975 ticks per 90 degrees

    private static final double     DRIVE_SPEED             = 0.45;
    private static final double     TURN_SPEED              = 0.45;
    private static boolean          driveFullPower          = false;
    private static double           motorKp                 = 0.005;
    private static double           motorKi                 = 0.0;
    private static double           motorKd                 = 0.0;
    private static double           motorRampTime           = 0.2;

    private static final double     ROBOT_INIT_POS_X    = 15.0;
    private static final double     ROBOT_INIT_POS_Y    = 15.0;
    private static final double     ROBOT_INIT_ANGLE    = 45.0;
    private static final float      mmPerInch        = 25.4f;

    private boolean allianceRed = false;


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private long startTime;

    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight, BNO055IMU imu, LinearOpMode opMode, ElapsedTime timer) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public double getAngularVMaxNeverrest20(){
        return ANGULAR_V_MAX_NEVERREST_20;
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

    public void checkAndStopMotors() {
        if (!frontLeft.isBusy()) { frontLeft.setPower(0); }
        if (!frontRight.isBusy()) { frontRight.setPower(0); }
        if (!rearLeft.isBusy()) { rearLeft.setPower(0); }
        if (!rearRight.isBusy()) { rearRight.setPower(0); }
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
     * Initialize MaxVelocity of drive motors
     */
    public void initMaxVelocity() {
        frontLeft.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        frontRight.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        rearLeft.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
        rearRight.setVelocity(ANGULAR_V_MAX_NEVERREST_20);
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
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

    // robot move in all directions
    public double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lfPower, rfPower, lrPower, rrPower};
    }

    // robot only move in forward/backward/left/right directions
    public double[] calcMotorPowers2(double leftStickX, double leftStickY, double rightStickX) {
        if(Math.abs(leftStickX) >= Math.abs((leftStickY))){
            leftStickY = 0;
        }
        else{
            leftStickX = 0;
        }
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lfPower, rfPower, lrPower, rrPower};
    }

    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void setDrivePowers(double[] powers) {
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        rearLeft.setPower(powers[2]);
        rearRight.setPower(powers[3]);
    }

    public void setDriveFullPower(boolean fullPower) {
        driveFullPower = fullPower;
    }

    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        rearLeft.setTargetPosition(targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    public int[] getCurrentPositions() {
        return new int[] {
                frontLeft.getCurrentPosition() - encoderOffsetFL,
                frontRight.getCurrentPosition() - encoderOffsetFR,
                rearLeft.getCurrentPosition() - encoderOffsetRL,
                rearRight.getCurrentPosition() - encoderOffsetRR
        };
    }

    public int[] getDriveMotorEncoders() {
        return new int[] {
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                rearRight.getCurrentPosition()
        };
    }

    public void resetDriveMotorEncoders() {
        encoderOffsetFL = frontLeft.getCurrentPosition();
        encoderOffsetFR = frontRight.getCurrentPosition();
        encoderOffsetRL = rearLeft.getCurrentPosition();
        encoderOffsetRR = rearRight.getCurrentPosition();
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

    public void turnRobotByTick(double angle) {
//        this.turnByTick(TURN_SPEED, angle);
        if (angle > 0.0) {
            allMotorPIDControl((int) (angle*COUNTS_PER_DEGREE), TURN_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                    motorRampTime, false, true, false, true, motorKp, motorKi, motorKd);
        }
        else {
            allMotorPIDControl((int) (-angle*COUNTS_PER_DEGREE), TURN_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                    motorRampTime, true, false, true, false, motorKp, motorKi, motorKd);
        }
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += angle;
        // Display it for the driver.
        opMode.telemetry.addData("turnRobot",  "turn to %7.2f degrees", robotCurrentAngle);
        opMode.telemetry.update();
//        opMode.sleep(100);
    }

    public void turnByTick(double power, double angle) {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);
        rearLeft.setTargetPosition(0);
        rearRight.setTargetPosition(0);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (driveFullPower) {
            setDrivePower(1.0);
        }
        else {
            setDrivePower(power);
        }
        // convert from degrees to motor counts
        int tickCount = (int) (angle * COUNTS_PER_DEGREE);
        frontLeft.setTargetPosition(-tickCount);
        frontRight.setTargetPosition(tickCount);
        rearLeft.setTargetPosition(-tickCount);
        rearRight.setTargetPosition(tickCount);
        startTime = timer.nanoseconds();
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {
            logDriveEncoders();
            checkAndStopMotors();
        }
        stop();
        logDriveEncoders();
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

    public double getYaw() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void turnByAngle(double power, double turnAngle) {
        double initialAngle = getYaw();
        double currentAngle;
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (turnAngle > 0.0) {
            // counter-clockwise
            currentAngle = initialAngle;
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
            currentAngle = initialAngle;
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
        opMode.telemetry.addData("initial angle",  "%7.2f degrees", initialAngle);
        opMode.telemetry.addData("last read angle",  "%7.2f degrees", currentAngle);
        opMode.telemetry.addData("final angle",  "%7.2f degrees", getYaw());
        opMode.sleep(3000);
        opMode.telemetry.addData("final2 angle",  "%7.2f degrees", getYaw());
        opMode.telemetry.update();
        opMode.sleep(3000);
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
        if (driveFullPower) {
            setPower2D(distanceCountX, distanceCountY, 1.0);
        }
        else {
            setPower2D(distanceCountX, distanceCountY, power);
        }
        setTargetPosition2D(distanceCountX, distanceCountY);
        startTime = timer.nanoseconds();
        while (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy()) {
            logDriveEncoders();
            checkAndStopMotors();
        }
        stop();
        logDriveEncoders();
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
//        frontLeft.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
//        frontRight.setTargetPosition((int) ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
//        rearLeft.setTargetPosition((int)   ((- targetPositionX + targetPositionY)*Math.sqrt(2.0)));
//        rearRight.setTargetPosition((int)  ((+ targetPositionX + targetPositionY)*Math.sqrt(2.0)));
        frontLeft.setTargetPosition((int)  (+ targetPositionX + targetPositionY));
        frontRight.setTargetPosition((int) (- targetPositionX + targetPositionY));
        rearLeft.setTargetPosition((int)   (- targetPositionX + targetPositionY));
        rearRight.setTargetPosition((int)  (+ targetPositionX + targetPositionY));
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
//        this.moveToPos2D(DRIVE_SPEED, 0.0, distance);
        allMotorPIDControl( (int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_Y), DRIVE_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, true, true, true, true, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveBackward(double distance) {
//        this.moveToPos2D(DRIVE_SPEED, 0.0, -distance);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_Y), DRIVE_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, false, false, false, false, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveLeft(double distance) {
//        this.moveToPos2D(DRIVE_SPEED, -distance, 0.0);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_X), DRIVE_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, false, true, true, false, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void moveRight(double distance) {
//        this.moveToPos2D(DRIVE_SPEED, distance, 0.0);
        allMotorPIDControl((int) (distance*COUNTS_PER_MM * COUNTS_CORRECTION_X), DRIVE_SPEED * ANGULAR_V_MAX_NEVERREST_20, ANGULAR_V_MAX_NEVERREST_20,
                motorRampTime, true, false, false, true, motorKp, motorKi, motorKd);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        opMode.telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        opMode.telemetry.update();
//        sleep(100);
    }

    public void park() {
        moveToPos2D(0.25,15,15); //temp position
        // change to where the tape is
    }

    public void printMotorPIDCoefficients() {
        PIDFCoefficients pidCoeff;
        pidCoeff = getMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Front Left ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Front Right", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Rear Left  ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        pidCoeff = getMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Rear Right ", "P: %.2f I: %.2f D: %.2f F: %.2f A: %s",
                pidCoeff.p, pidCoeff.i, pidCoeff.d, pidCoeff.f, pidCoeff.algorithm.toString());
        opMode.telemetry.update();
    }

    public void setMotorKp(double motorKPFL, double motorKPFR, double motorKPRL, double motorKPRR) {
        frontLeft.setPositionPIDFCoefficients(motorKPFL);
        frontRight.setPositionPIDFCoefficients(motorKPFR);
        rearLeft.setPositionPIDFCoefficients(motorKPRL);
        rearRight.setPositionPIDFCoefficients(motorKPRR);
    }

    public void setMotorPID(double Kp, double Ki, double Kd, double Kf) {
        PIDFCoefficients pidCoeff = new PIDFCoefficients();
        pidCoeff.p = Kp;
        pidCoeff.i = Ki;
        pidCoeff.d = Kd;
        pidCoeff.f = Kf;
        pidCoeff.algorithm = MotorControlAlgorithm.PIDF;
        setMotorPIDCoefficients(frontLeft, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
        setMotorPIDCoefficients(frontRight, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
        setMotorPIDCoefficients(rearLeft, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
        setMotorPIDCoefficients(rearRight, DcMotor.RunMode.RUN_TO_POSITION, pidCoeff);
    }

    public PIDFCoefficients getMotorPIDCoefficients(DcMotorEx motor, DcMotor.RunMode mode) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = motor.getPortNumber();

        // get the PID coefficients for the specific motor mode.
        PIDFCoefficients pidOrig = motorControllerEx.getPIDFCoefficients(motorIndex, mode);

        return pidOrig;
    }

    public void setMotorPIDCoefficients(DcMotorEx motor, DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx) motor.getController();

        // get the port number of our configured motor.
        int motorIndex = motor.getPortNumber();

        // get the PID coefficients for the specific motor mode.
        motorControllerEx.setPIDFCoefficients(motorIndex, mode, pidfCoefficients);
    }

    public void logDriveEncoders() {
        long currentTime1 = timer.nanoseconds();
        int[] encoders = getDriveMotorEncoders();
        long currentTime2 = timer.nanoseconds();
        long currentTime3 = timer.nanoseconds();
        String output = String.format("%d, %d, %d, FL %d, FR %d, RL %d, RR %d", currentTime1 - startTime, currentTime2-currentTime1, currentTime3-currentTime2,
                encoders[0], encoders[1], encoders[2], encoders[3]);
        Log.d("motorEnc", output);
    }

    /**
     * PID motor control program to ensure all four motors are synchronized
     * @param tickCount: absolute value of target tickcount of motor
     * @param peakSpeed: peak speed of motor rotation in tick per second
     * @param maxSpeed: max speed of motor rotation in tick per second
     * @param rampTime: motor speed ramp up/down time in sec
     * @param motorFLForward: front left motor is forward
     * @param motorFRForward: front right motor is forward
     * @param motorRLForward: rear left motor is forward
     * @param motorRRForward: rear right motor is forward
     * @param Kp: coefficient Kp
     * @param Ki: coefficient Ki
     * @param Kd: coefficient Kd
     */
    public void allMotorPIDControl(int tickCount, double peakSpeed, double maxSpeed, double rampTime,
                                   boolean motorFLForward, boolean motorFRForward, boolean motorRLForward, boolean motorRRForward,
                                   double Kp, double Ki, double Kd) {
        stop();
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean isMotorFLDone = false;
        boolean isMotorFRDone = false;
        boolean isMotorRLDone = false;
        boolean isMotorRRDone = false;
        double acculErrorFL = 0.0;
        double acculErrorFR = 0.0;
        double acculErrorRL = 0.0;
        double acculErrorRR = 0.0;
        double prevErrorFL = 0.0;
        double prevErrorFR = 0.0;
        double prevErrorRL = 0.0;
        double prevErrorRR = 0.0;
        double prevTimeFL = 0.0;
        double prevTimeFR = 0.0;
        double prevTimeRL = 0.0;
        double prevTimeRR = 0.0;
        boolean initialized = false;        // disable Kd term in first iteration
        int currentCount, targetCount;
        double currentError = 0.0;
        double currentTargetSpeed;
        double currentPower;
        double alpha = 0.95;
        double startTime = ((double) timer.nanoseconds()) * 1.0e-9;
        double currentTime;
        while ((!isMotorFLDone) || (!isMotorFRDone) || (!isMotorRLDone) || (!isMotorRRDone)) {
            if (!isMotorFLDone) {
                currentCount = frontLeft.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime-startTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime-startTime);
                if (motorFLForward) {
                    if (currentCount >= tickCount) {
                        isMotorFLDone = true;
                        frontLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        acculErrorFL = acculErrorFL*alpha + currentError;
                        currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorFL*Ki;
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        frontLeft.setPower(currentPower);
                    }
                }
                else { // motorFLForward is false
                    if (currentCount <= -tickCount) {
                        isMotorFLDone = true;
                        frontLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        acculErrorFL = acculErrorFL*alpha + currentError;
                        currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorFL*Ki;
                        if (currentPower < 1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        frontLeft.setPower(currentPower);
                    }
                }
                prevErrorFL = currentError;
                prevTimeFL = currentTime;
            } // if (!isMotorFLDone)
            if (!isMotorFRDone) {
                currentCount = frontRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime-startTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime-startTime);
                if (motorFRForward) {
                    if (currentCount >= tickCount) {
                        isMotorFRDone = true;
                        frontRight.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        acculErrorFR = acculErrorFR*alpha + currentError;
                        currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorFR*Ki;
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        frontRight.setPower(currentPower);
                    }
                }
                else { // motorFRForward is false
                    if (currentCount <= -tickCount) {
                        isMotorFRDone = true;
                        frontRight.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        acculErrorFR = acculErrorFR*alpha + currentError;
                        currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorFR*Ki;
                        if (currentPower < 1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        frontRight.setPower(currentPower);
                    }
                }
                prevErrorFR = currentError;
                prevTimeFR = currentTime;
            } // if (!isMotorFRDone)
            if (!isMotorRLDone) {
                currentCount = rearLeft.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime-startTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime-startTime);
                if (motorRLForward) {
                    if (currentCount >= tickCount) {
                        isMotorRLDone = true;
                        rearLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        acculErrorRL = acculErrorRL*alpha + currentError;
                        currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorRL*Ki;
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                }
                else { // motorFLForward is false
                    if (currentCount <= -tickCount) {
                        isMotorRLDone = true;
                        rearLeft.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        acculErrorRL = acculErrorRL*alpha + currentError;
                        currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorRL*Ki;
                        if (currentPower < 1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        rearLeft.setPower(currentPower);
                    }
                }
                prevErrorRL = currentError;
                prevTimeRL = currentTime;
            } // if (!isMotorRLDone)
            if (!isMotorRRDone) {
                currentCount = rearRight.getCurrentPosition();
                currentTime = ((double) timer.nanoseconds()) * 1.0e-9;
                targetCount = getTargetTickCount(tickCount, peakSpeed, rampTime, currentTime-startTime);
                currentTargetSpeed = getTargetSpeed(tickCount, peakSpeed, rampTime, currentTime-startTime);
                if (motorRRForward) {
                    if (currentCount >= tickCount) {
                        isMotorRRDone = true;
                        rearRight.setPower(0.0);
                    }
                    else {
                        currentError = (double) (currentCount-targetCount);
                        acculErrorRR = acculErrorRR*alpha + currentError;
                        currentPower = currentTargetSpeed/maxSpeed - currentError*Kp - acculErrorRR*Ki;
                        if (currentPower > 1.0) currentPower = 1.0;
                        if (currentPower < 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                }
                else { // motorFLForward is false
                    if (currentCount <= -tickCount) {
                        isMotorRRDone = true;
                        rearRight.setPower(0.0);
                    }
                    else {
                        currentError = (double) (-currentCount-targetCount);
                        acculErrorRR = acculErrorRR*alpha + currentError;
                        currentPower = -currentTargetSpeed/maxSpeed + currentError*Kp + acculErrorRR*Ki;
                        if (currentPower < 1.0) currentPower = -1.0;
                        if (currentPower > 0.0) currentPower = 0.0;
                        rearRight.setPower(currentPower);
                    }
                }
                prevErrorRR = currentError;
                prevTimeRR = currentTime;
            } // if (!isMotorRRDone)
            initialized = true;         // enable Kd term
        }

    }

    private int getTargetTickCount(int tickCount, double speed, double rampTime, double elapsedTime) {
        int targetTick;
        double tickCountD = (double) tickCount;
        if (elapsedTime < rampTime) { // during ramp up time
            targetTick = (int) (0.5*speed * elapsedTime * elapsedTime/rampTime);
        }
        else if (tickCountD > speed * elapsedTime) { // during constant speed period
            targetTick = (int) (speed * (elapsedTime-rampTime*0.5));
        }
        else {  // during ramp down time
            double remainTime = tickCountD/speed + rampTime - elapsedTime;
            targetTick = tickCount - ((int) (0.5*remainTime*speed*remainTime/rampTime));
        }
        if (targetTick > tickCount) targetTick = tickCount;
        return targetTick;
    }

    private double getTargetSpeed(int tickCount, double speed, double rampTime, double elapsedTime) {
        double targetSpeed;
        double tickCountD = (double) tickCount;
        if (elapsedTime < rampTime) { // during ramp up time
            targetSpeed = speed * elapsedTime/rampTime;
        }
        else if (tickCountD > speed * elapsedTime) { // during constant speed period
            targetSpeed = speed;
        }
        else {  // during ramp down time
            double remainTime = tickCountD/speed + rampTime - elapsedTime;
            targetSpeed = speed - speed*remainTime/rampTime;
        }
        return targetSpeed;
    }
}