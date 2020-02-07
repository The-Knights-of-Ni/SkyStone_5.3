package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Control subsystem for controlling arms and claws
 * Created by AndrewC on 1/17/2020
 */
public class Control extends Subsystem {
    // device declaration
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    //DC Motors
    private DcMotorEx xRailWinch;
    private DcMotorEx armTilt;

    //Servos
    private Servo mainClawArm;
    private Servo mainClawRotation;
    private Servo mainClaw; //0
    private Servo csClaw; //capstone claw
    private Servo csArm; //capstone arm
    private Servo fClawL; //foundationClawLeft
    private Servo fClawR; // foundationClawRight

    //Sensors
    private BNO055IMU imu;

    // define physical/structural constants

    /**
     * The TILT_TABLE is a lookup table for mapping the main arm angle to the arm tilt motor tick
     * The TILT_TABLE_SIZE records the number of entries in the TILT_TABLE
     * The TILT_TABLE consists of TILT_TABLE_SIZE pairs of data. Each pair is (tilt angle, arm tilt motor tick).
     */
    private static final int        TILT_TABLE_SIZE                     = 83;
    private static final double[]   TILT_TABLE = {
            0.0, 0, 0.6, 88, 1.5, 131, 2.2, 160, 2.9, 200, 3.4, 223, 4.1, 257, 5.0, 301, 5.8, 350, 6.7, 392,
            7.3, 432, 8.2, 479, 9.1, 535, 9.7, 571, 10.5, 617, 11.2, 658, 11.7, 686, 12.2, 716, 12.7, 748, 13.2, 781,
            13.6, 810, 14.1, 838, 14.7, 880, 15.2, 918, 15.8, 951, 16.1, 972, 16.9, 1023, 17.9, 1093, 18.5, 1131, 18.9, 1158,
            19.5, 1205, 20.2, 1252, 20.9, 1300, 21.3, 1333, 21.9, 1376, 22.7, 1428, 23.3, 1473, 23.9, 1518, 25.0, 1602, 25.9, 1666,
            26.9, 1736, 27.8, 1801, 28.5, 1854, 29.5, 1932, 30.3, 1997, 31.2, 2067, 32.0, 2120, 33.0, 2199, 34.0, 2269, 34.7, 2336,
            35.5, 2391, 36.5, 2470, 37.2, 2534, 38.2, 2608, 38.9, 2661, 40.1, 2750, 40.8, 2812, 41.4, 2862, 42.2, 2923, 43.1, 3000,
            44.0, 3060, 45.1, 3152, 45.9, 3210, 46.7, 3284, 47.8, 3372, 48.7, 3444, 49.4, 3501, 50.5, 3584, 51.0, 3630, 51.9, 3700,
            53.3, 3804, 54.4, 3893, 55.5, 3987, 57.0, 4106, 58.1, 4186, 59.6, 4301, 61.0, 4414, 62.0, 4491, 63.0, 4564, 63.7, 4615,
            64.7, 4688, 65.9, 4771, 67.5, 4889 };

    /**
     * The CLAW_ARM_TILT_TABLE is a lookup table for mapping the claw arm angle to the claw arm servo
     * The CLAW_ARM_TILT_TABLE_SIZE records the number of entries in the CLAW_ARM_TILT_TABLE
     * The CLAW_ARM_TILT_TABLE consists of CLAW_ARM_TILT_TABLE_SIZE pairs of data. Each pair is (tilt angle, arm tilt servo).
     */
    private static final int        CLAW_ARM_TILT_TABLE_SIZE                     = 24;
    private static final double[]   CLAW_ARM_TILT_TABLE = {
            -183.0, 0.01, -180.0, 0.04, -70.2, 0.416, -30.8, 0.563, -12.2, 0.634, -5.8, 0.658, -2.9, 0.679, 0.2, 0.692, 4.6, 0.703, 6.1, 0.716,
            11.0, 0.736, 15.2, 0.743, 18.2, 0.763, 23.2, 0.782, 29.0, 0.804, 33.9, 0.825, 40.8, 0.851, 46.6, 0.879, 51.0, 0.894, 57.4, 0.921,
            62.5, 0.941, 70.1, 0.971, 75.6, 0.993, 77.6, 1.0 };

    //DO WITH ENCODERS
    private static final double     TICKS_PER_MOTOR_REV_40          = 1120;    // AM Orbital 20 motor
    private static final double     RPM_MAX_NEVERREST_40            = 160;
    private static final double     ANGULAR_V_MAX_NEVERREST_40      = (TICKS_PER_MOTOR_REV_40 * RPM_MAX_NEVERREST_40) / 60.0;

    private static final double     WINCH_DIAMETER_INCH                 = 1.244;  //inch original measurement
    private static final double     WINCH_DIAMETER_MM                   = WINCH_DIAMETER_INCH * 2.54 * 10.0; //milimeters
    private static final double     WINCH_RADIUS_MM                     = WINCH_DIAMETER_MM / 2.0;
    private static final double     WINCH_CIRCUMFERENCE_MM              = WINCH_RADIUS_MM * 2.0 * Math.PI;
    private static final double     MOTOR_TICK_PER_REV_NEVERREST40      = 1120.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET223   = 753.2;
    private static final double     REV_PER_MIN_YELLOJACKET223          = 223.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET1620   = 103.6;
    private static final double     REV_PER_MIN_YELLOJACKET1620          = 1620.0;
    private static final double     WINCH_MAX_SPEED_MM_PER_SEC          = (RPM_MAX_NEVERREST_40 * WINCH_DIAMETER_MM * Math.PI) / 60.0;
    private static final double     WINCH_MAX_SPEED_TICK_PER_SEC        = (MOTOR_TICK_PER_REV_NEVERREST40 * RPM_MAX_NEVERREST_40) / 60.0;
    private static final double     TILT_MAX_SPEED_TICK_PER_SEC         = (MOTOR_TICK_PER_REV_YELLOJACKET1620 * REV_PER_MIN_YELLOJACKET1620) / 60.0;
//    private static final double     TILT_TICK_PER_90_DEGREE             = 2510.0;
    private static final double     WINCH_MM_PER_TICK                   = WINCH_CIRCUMFERENCE_MM / MOTOR_TICK_PER_REV_NEVERREST40;

    private static final double     MAINARM_INIT_LENGTH          = 371.0;    // main arm length before extending (mm)
    private static final double     MAINARM_STACK_HEIGHT         = 123.0;    // main arm vertical offset (mm)

    private static final double     MAINARM_LENGTH_TICK_MAX         = 8900.0;    // main arm tick max

    // Servos
    private static final double     fClawLFoundation = 0.42;
    private static final double     fClawRFoundation = 0.58;
    private static final double     fClawLDown = 0.36;
    private static final double     fClawLUp = 0.795;
    private static final double     fClawRUp = 0.05;
    private static final double     fClawRDown = 0.63;

    private static final double     CLAW_ARM_POS_ANGLE                  = 70.1; // most positive angle
    private static final double     CLAW_ARM_POS_VALUE                  = 0.971; // servo setting at most positive angle
    private static final double     CLAW_ARM_POS_0_DEG                  = 0.692; // xRail horizontal and main claw facing down
    private static final double     CLAW_ARM_POS_N180_DEG                = 0.04;
    private static final double     CLAW_ARM_ROT_0_DEG                  = 0.046;
    private static final double     CLAW_ARM_ROT_180_DEG                = 0.796;
    private static final double     MAIN_CLAW_POS_OPEN_WIDE              = 0.369;
    private static final double     MAIN_CLAW_POS_OPEN                  = 0.416;
    private static final double     MAIN_CLAW_POS_CLOSED_STONE          = 0.589;
    private static final double     MAIN_CLAW_POS_CLOSED                = 0.64;

    private static final double     CS_ARM_POS_ANGLE                  = 68.5; // most positive angle
    private static final double     CS_ARM_POS_VALUE                  = 0.038; // servo setting at most positive angle
    private static final double     CS_ARM_POS_0_DEG                  = 0.301; // xRail horizontal and cs claw facing down
    private static final double     CS_ARM_POS_N180_DEG                = 0.939;
    private static final double     CS_CLAW_POS_OPEN                  = 0.66;
    private static final double     CS_CLAW_POS_CLOSED                = 0.43;

    // define variables
    private double mainArmAngle = 0.0;
    private double mainArmTargetAngle = 0.0;
    private double mainArmAngleTick = 0.0;
    private double mainArmLength = 0.0;
    private double mainArmLengthTick = 0.0;
    private boolean mainClawArmTrackingMode = false;
    private double mainClawRotationAngle = 0.0;

    public Control(DcMotorEx xRailWinch, DcMotorEx armTilt, Servo mainClaw, Servo mainClawRotation, Servo mainClawArm,
                   Servo csClaw, Servo csArm, Servo fClawL, Servo fClawR, BNO055IMU imu, LinearOpMode opMode, ElapsedTime timer) {
        // store device information locally
        this.xRailWinch = xRailWinch;
        this.armTilt = armTilt;
        this.mainClaw = mainClaw;
        this.mainClawRotation = mainClawRotation;
        this.mainClawArm = mainClawArm;
        this.csClaw = csClaw;
        this.csArm = csArm;
        this.fClawL = fClawL;
        this.fClawR = fClawR;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize main arm parameters
        mainClawArmTrackingMode = false;
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        xRailWinch.setZeroPowerBehavior(mode);
        armTilt.setZeroPowerBehavior(mode);
    }

    // main arm tilting angle from horizontal
    public double getMainArmAngle() {
        mainArmAngle = mainArmTickToAngle((double) armTilt.getCurrentPosition());
        return mainArmAngle;
    }
    public double getMainArmTargetAngle() {
        return mainArmTargetAngle;
    }
    public void setMainArmAngle(double angle) {
        mainArmTargetAngle = angle;
        mainArmAngleTick = mainArmAngleToTick(angle);
        armTilt.setTargetPosition((int) mainArmAngleTick);
    }
    public double getMainArmAngleTickTarget() { return mainArmAngleTick;}
    public double getMainArmAngleTickCurrent() { return (double) armTilt.getCurrentPosition();}

    // main arm extension length
    public double getMainArmExtensionLength() {
        return WINCH_MM_PER_TICK * ((double) xRailWinch.getCurrentPosition());
    }
    public void setMainArmExtensionLength(double length) {
        mainArmLengthTick = length / WINCH_MM_PER_TICK;
        if (mainArmLengthTick > MAINARM_LENGTH_TICK_MAX) {
            mainArmLengthTick = MAINARM_LENGTH_TICK_MAX;
        }
        xRailWinch.setTargetPosition((int) mainArmLengthTick);
    }
    public double getMainArmLengthTickTarget() { return mainArmLengthTick;}
    public double getMainArmLengthTickCurrent() { return (double) xRailWinch.getCurrentPosition();}

    // main arm total length
    public double getMainArmTotalLength() {
        return getMainArmExtensionLength() + MAINARM_INIT_LENGTH;
    }
    public void setMainArmTotalLength(double length) {
        setMainArmExtensionLength(length - MAINARM_INIT_LENGTH);
    }

    // main arm horizontal offset measured from the initial position
    public double getMainArmHorizontalOffset() {
        mainArmAngle = getMainArmAngle();
        return getMainArmTotalLength()*Math.cos(mainArmAngle*Math.PI/180.0)
                - MAINARM_STACK_HEIGHT*Math.sin(mainArmAngle*Math.PI/180.0) - MAINARM_INIT_LENGTH;
    }

    // main arm vertical offset measured from the initial position
    public double getMainArmVerticalOffset() {
        mainArmAngle = getMainArmAngle();
        return getMainArmTotalLength()*Math.sin(mainArmAngle*Math.PI/180.0)
                + MAINARM_STACK_HEIGHT*Math.cos(mainArmAngle*Math.PI/180.0) - MAINARM_STACK_HEIGHT;
    }

    // set main arm offset measured from the initial position
    public void setMainArmPosition(double horizontalOffset, double verticalOffset) {
        double targetAngle, targetLength;
        // change coordinate system to the main arm pivoting point
        double targetHorizontalOffset = horizontalOffset + MAINARM_INIT_LENGTH;
        double targetVerticalOffset = verticalOffset + MAINARM_STACK_HEIGHT;
        if (targetHorizontalOffset > targetVerticalOffset) {
            // use targetHorizontalOffset as the anchor formula
            // initial guess of target length
            targetLength = Math.sqrt(targetHorizontalOffset*targetHorizontalOffset + targetVerticalOffset*targetVerticalOffset);
            // first iteration
            targetAngle = (Math.atan(targetVerticalOffset/targetHorizontalOffset)
                            - Math.acos(targetLength/Math.sqrt(targetLength*targetLength+MAINARM_STACK_HEIGHT*MAINARM_STACK_HEIGHT)))*180.0/Math.PI;
            targetLength = (targetHorizontalOffset + MAINARM_STACK_HEIGHT*Math.sin(targetAngle*Math.PI/180.0)) / Math.cos(targetAngle*Math.PI/180.0);
            // second iteration
            targetAngle = (Math.atan(targetVerticalOffset/targetHorizontalOffset)
                    - Math.acos(targetLength/Math.sqrt(targetLength*targetLength+MAINARM_STACK_HEIGHT*MAINARM_STACK_HEIGHT)))*180.0/Math.PI;
            targetLength = (targetHorizontalOffset + MAINARM_STACK_HEIGHT*Math.sin(targetAngle*Math.PI/180.0)) / Math.cos(targetAngle*Math.PI/180.0);
            // third iteration
            targetAngle = (Math.atan(targetVerticalOffset/targetHorizontalOffset)
                    - Math.acos(targetLength/Math.sqrt(targetLength*targetLength+MAINARM_STACK_HEIGHT*MAINARM_STACK_HEIGHT)))*180.0/Math.PI;
            targetLength = (targetHorizontalOffset + MAINARM_STACK_HEIGHT*Math.sin(targetAngle*Math.PI/180.0)) / Math.cos(targetAngle*Math.PI/180.0);
        }
        else {
            // use targetVerticalOffset as the anchor formula
            // initial guess of target length
            targetLength = Math.sqrt(targetHorizontalOffset*targetHorizontalOffset + targetVerticalOffset*targetVerticalOffset);
            // first iteration
            targetAngle = (Math.atan(targetVerticalOffset/targetHorizontalOffset)
                    - Math.acos(targetLength/Math.sqrt(targetLength*targetLength+MAINARM_STACK_HEIGHT*MAINARM_STACK_HEIGHT)))*180.0/Math.PI;
            targetLength = (targetVerticalOffset - MAINARM_STACK_HEIGHT*Math.cos(targetAngle*Math.PI/180.0)) / Math.sin(targetAngle*Math.PI/180.0);
            // second iteration
            targetAngle = (Math.atan(targetVerticalOffset/targetHorizontalOffset)
                    - Math.acos(targetLength/Math.sqrt(targetLength*targetLength+MAINARM_STACK_HEIGHT*MAINARM_STACK_HEIGHT)))*180.0/Math.PI;
            targetLength = (targetVerticalOffset - MAINARM_STACK_HEIGHT*Math.cos(targetAngle*Math.PI/180.0)) / Math.sin(targetAngle*Math.PI/180.0);
            // third iteration
            targetAngle = (Math.atan(targetVerticalOffset/targetHorizontalOffset)
                    - Math.acos(targetLength/Math.sqrt(targetLength*targetLength+MAINARM_STACK_HEIGHT*MAINARM_STACK_HEIGHT)))*180.0/Math.PI;
            targetLength = (targetVerticalOffset - MAINARM_STACK_HEIGHT*Math.cos(targetAngle*Math.PI/180.0)) / Math.sin(targetAngle*Math.PI/180.0);
        }

        setMainArmAngle(targetAngle);
        setMainArmTotalLength(targetLength);
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
    public double getAngularVMaxNeverrest40(){
        return ANGULAR_V_MAX_NEVERREST_40;
    }
    public double getMotorTickPerRevYellojacket223(){
        return MOTOR_TICK_PER_REV_YELLOJACKET223;
    }
//    public double getTiltTickPer90Degree(){
//        return TILT_TICK_PER_90_DEGREE;
//    }
    public double getClawArmPos0Deg(){
        return CLAW_ARM_POS_0_DEG;
    }
    public double getClawArmPosN180Deg(){
        return CLAW_ARM_POS_N180_DEG;
    }
    public double getClawArmRot0Deg(){
        return CLAW_ARM_ROT_0_DEG;
    }
    public double getClawArmRot180Deg(){
        return CLAW_ARM_ROT_180_DEG;
    }
    public double getMainClawPosOpenWide(){
        return  MAIN_CLAW_POS_OPEN_WIDE;
    }
    public double getMainClawPosOpen(){
        return  MAIN_CLAW_POS_OPEN;
    }
    public double getMainClawPosClosedStone(){
        return MAIN_CLAW_POS_CLOSED_STONE;
    }
    public double getMainClawPosClosed(){
        return MAIN_CLAW_POS_CLOSED;
    }
    public double getCSArmPos0Deg(){
        return CS_ARM_POS_0_DEG;
    }
    public double getCSArmPosN180Deg(){
        return CS_ARM_POS_N180_DEG;
    }
    public double getCSClawPosOpen(){
        return  CS_CLAW_POS_OPEN;
    }
    public double getCSClawPosClosed(){
        return CS_CLAW_POS_CLOSED;
    }

    public void retractMainClawArm() {
        setMainClawArmDegrees(-180.0);
    }

    public void retractCSClawArm() {
        setCSClawArmDegrees(-174.0);
    }


    public void lowerClawsToFoundation() {
        fClawL.setPosition(fClawLFoundation);
        fClawR.setPosition(fClawRFoundation);
    }

    public void raiseClawsFromFoundation() {
        fClawL.setPosition(fClawLUp);
        fClawR.setPosition(fClawRUp);
    }

//    public void pickUpStone() {
//        // Lower arm
//        // need to add rotation, arm claw synchronization
//        mainClawArm.setPosition(mainArmDown);
//        mainClaw.setPosition(mainClawOpen);
//        mainClaw.setPosition(mainClawStone);
//        mainClawArm.setPosition(0.5);
//    }
//
//    public void dropStone() {
//        // lower arm
//        // rotate?
//        mainClawArm.setPosition(mainArmDown);
//        mainClaw.setPosition(mainClawOpen);
//        mainClawArm.setPosition(0.5);
//        mainClaw.setPosition(0);
//    }
//
    public void openMainClawWide() {
    mainClaw.setPosition(this.getMainClawPosOpenWide());
}
    public void openMainClaw() {
        mainClaw.setPosition(this.getMainClawPosOpen());
    }
    public void closeMainClawStone() {
        mainClaw.setPosition(this.getMainClawPosClosedStone());
    }
    public void closeMainClaw() {
        mainClaw.setPosition(this.getMainClawPosClosed());
    }
    public void setMainClawArmDegrees(double angle) {
        mainClawArm.setPosition(this.mainClawArmAngleToPos(angle));
    }
    public double mainClawArmAngleToPos(double angle){
//        double value;
//        if (angle < 0.0) {
//            value = ((angle / 180.0) * (CLAW_ARM_POS_0_DEG - CLAW_ARM_POS_N180_DEG)) + CLAW_ARM_POS_0_DEG;
//        }
//        else {
//            value = ((angle / CLAW_ARM_POS_ANGLE) * (CLAW_ARM_POS_VALUE - CLAW_ARM_POS_0_DEG)) + CLAW_ARM_POS_0_DEG;
//        }
//        if (value > 1.0) value = 1.0;
//        if (value < 0.0) value = 0.0;
//        return value;
//
        int lowerIndex, upperIndex;
        int i = 1;
        double servoTarget;
        while ((i < CLAW_ARM_TILT_TABLE_SIZE) && (CLAW_ARM_TILT_TABLE[i*2] < angle)) {
            ++i;
        }
        upperIndex = i;
        lowerIndex = i-1;
        servoTarget = CLAW_ARM_TILT_TABLE[lowerIndex*2+1] +
                (CLAW_ARM_TILT_TABLE[upperIndex*2+1]-CLAW_ARM_TILT_TABLE[lowerIndex*2+1])*(angle-CLAW_ARM_TILT_TABLE[lowerIndex*2])
                        /(CLAW_ARM_TILT_TABLE[upperIndex*2]-CLAW_ARM_TILT_TABLE[lowerIndex*2]);
        return servoTarget;
    }
    public void setMainClawRotationDegrees(double angle) {
        mainClawRotationAngle = angle;
        mainClawRotation.setPosition(this.mainClawRotationAngleToPos(angle));
    }
    public double getMainClawRotationDegrees() {
        return mainClawRotationAngle;
    }
    public double mainClawRotationAngleToPos(double angle){
        double value = ((angle / 180.0) * (this.getClawArmRot180Deg() - this.getClawArmRot0Deg())) + this.getClawArmRot0Deg();
        if (value > 1.0) value = 1.0;
        if (value < 0.0) value = 0.0;
        return value;
    }

    public void openCSClaw() {
        csClaw.setPosition(this.getCSClawPosOpen());
    }
    public void closeCSClaw() {
        csClaw.setPosition(this.getCSClawPosClosed());
    }
    public void setCSClawArmDegrees(double angle) {
        csArm.setPosition(this.CSClawArmAngleToPos(angle));
    }
    public double CSClawArmAngleToPos(double angle){
        double value;
        if (angle < 0.0) {
            value = ((angle / 180.0) * (CS_ARM_POS_0_DEG - CS_ARM_POS_N180_DEG)) + CS_ARM_POS_0_DEG;
        }
        else {
            value = ((angle / CS_ARM_POS_ANGLE) * (CS_ARM_POS_VALUE - CS_ARM_POS_0_DEG)) + CS_ARM_POS_0_DEG;
        }
        if (value > 1.0) value = 1.0;
        if (value < 0.0) value = 0.0;
        return value;
    }

    public void modifyServo(Servo servo, double value) {
        double currentValue = servo.getPosition();
        currentValue = currentValue + value;
        if (currentValue > 1.0) currentValue = 1.0;
        if (currentValue < 0.0) currentValue = 0.0;
        servo.setPosition(currentValue);
    }

    /**
     * look up motor tick count position from TILT_TABLE using main arm tilting angle
     * @param angle
     * @return motor tick count position
     */
    public double mainArmAngleToTick(double angle) {
        int lowerIndex, upperIndex;
        int i = 1;
        double tickTarget;
        while ((i < TILT_TABLE_SIZE) && (TILT_TABLE[i*2] < angle)) {
            ++i;
        }
        upperIndex = i;
        lowerIndex = i-1;
        tickTarget = TILT_TABLE[lowerIndex*2+1] +
                (TILT_TABLE[upperIndex*2+1]-TILT_TABLE[lowerIndex*2+1])*(angle-TILT_TABLE[lowerIndex*2])
                        /(TILT_TABLE[upperIndex*2]-TILT_TABLE[lowerIndex*2]);
        return tickTarget;
    }

    /**
     * look up main arm tiling angle from TILT_TABLE using motor tick count
     * @param tick
     * @return main arm tilting angle
     */
    public double mainArmTickToAngle(double tick) {
        int lowerIndex, upperIndex;
        int i = 1;
        double angleTarget;
        while ((i < TILT_TABLE_SIZE) && (TILT_TABLE[i*2+1] < tick)) {
            ++i;
        }
        upperIndex = i;
        lowerIndex = i-1;
        angleTarget = TILT_TABLE[lowerIndex*2] +
                (TILT_TABLE[upperIndex*2]-TILT_TABLE[lowerIndex*2])*(tick-TILT_TABLE[lowerIndex*2+1])
                        /(TILT_TABLE[upperIndex*2+1]-TILT_TABLE[lowerIndex*2+1]);
        return angleTarget;
    }


}
