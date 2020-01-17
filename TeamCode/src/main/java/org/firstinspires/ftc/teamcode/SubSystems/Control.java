package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
    private OpMode opMode;
    private HardwareMap hardwareMap;
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

    /**
     * The TILT_TABLE is a lookup table for mapping the main arm angle to the arm tilt motor tick
     * The TILT_TABLE_SIZE records the number of entries in the TILT_TABLE
     * The TILT_TABLE consists of TILT_TABLE_SIZE pairs of data. Each pair is (tilt angle, arm tilt motor tick).
     */
    private static final int        TILT_TABLE_SIZE                     = 50;
    private static final double[]   TILT_TABLE = {0.0, 10, 1.2, 20};

    //DO WITH ENCODERS
    private static final double     TICKS_PER_MOTOR_REV_40          = 1120;    // AM Orbital 20 motor
    private static final double     RPM_MAX_NEVERREST_40            = 160;
    private static final double     ANGULAR_V_MAX_NEVERREST_40      = (TICKS_PER_MOTOR_REV_40 * RPM_MAX_NEVERREST_40) / 60.0;

    private static final double     WINCH_DIAMETER_INCH                 = 1.244;  //inch original measurement
    private static final double     WINCH_DIAMETER_MM                   = WINCH_DIAMETER_INCH * 2.54 * 10.0; //milimeters
    private static final double     WINCH_RADIUS_MM                     = WINCH_DIAMETER_MM / 2.0;
    private static final double     WINCH_CIRCUMFERENCE_MM              = WINCH_RADIUS_MM * 2.0 * Math.PI;
    private static final double     MOTOR_TICK_PER_REV_NEVERREST40      = 1120.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET223   = 188.3;
    private static final double     REV_PER_MIN_YELLOJACKET223          = 223.0;
    private static final double     MOTOR_TICK_PER_REV_YELLOJACKET1620   = 25.9;
    private static final double     REV_PER_MIN_YELLOJACKET1620          = 1620.0;
    private static final double     WINCH_MAX_SPEED_MM_PER_SEC          = (160.0 * WINCH_DIAMETER_MM * Math.PI) / 60.0;
    private static final double     WINCH_MAX_SPEED_TICK_PER_SEC        = (MOTOR_TICK_PER_REV_NEVERREST40 * 160.0) / 60.0;
    private static final double     TILT_MAX_SPEED_TICK_PER_SEC         = (MOTOR_TICK_PER_REV_YELLOJACKET223 * REV_PER_MIN_YELLOJACKET223) / 60.0;
    private static final double     TILT_TICK_PER_90_DEGREE             = 2510.0;

    // Servos
    private static final double     fClawLFoundation = 0.45;
    private static final double     fClawRFoundation = 0.56;
    private static final double     fClawLDown = 0.36;
    private static final double     fClawLUp = 0.795;
    private static final double     fClawRUp = 0.21;
    private static final double     fClawRDown = 0.63;

    private static final double     CLAW_ARM_POS_0_DEG                  = 0.13; // xRail horizontal and main claw facing down
    private static final double     CLAW_ARM_POS_180_DEG                = 0.88;
    private static final double     CLAW_ARM_ROT_0_DEG                  = 0.046;
    private static final double     CLAW_ARM_ROT_180_DEG                = 0.796;
    private static final double     MAIN_CLAW_POS_OPEN                  = 0.65;
    private static final double     MAIN_CLAW_POS_CLOSED_STONE          = 0.35;
    private static final double     MAIN_CLAW_POS_CLOSED                = 0.35;

    private static final double     CS_ARM_POS_0_DEG                  = 0.833; // xRail horizontal and cs claw facing down
    private static final double     CS_ARM_POS_180_DEG                = 0.142;
    private static final double     CS_CLAW_POS_OPEN                  = 0.66;
    private static final double     CS_CLAW_POS_CLOSED                = 0.43;

    public Control(DcMotorEx xRailWinch, DcMotorEx armTilt, Servo mainClaw, Servo mainClawRotation, Servo mainClawArm,
                   Servo csClaw, Servo csArm, Servo fClawL, Servo fClawR, BNO055IMU imu, ElapsedTime timer, OpMode opMode) {
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
    }

    /**
     * Sets all drive motors to specified zero power behavior
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        xRailWinch.setZeroPowerBehavior(mode);
        armTilt.setZeroPowerBehavior(mode);
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
    public double getTiltTickPer90Degree(){
        return TILT_TICK_PER_90_DEGREE;
    }
    public double getClawArmPos0Deg(){
        return CLAW_ARM_POS_0_DEG;
    }
    public double getClawArmPos180Deg(){
        return CLAW_ARM_POS_180_DEG;
    }
    public double getClawArmRot0Deg(){
        return CLAW_ARM_ROT_0_DEG;
    }
    public double getClawArmRot180Deg(){
        return CLAW_ARM_ROT_180_DEG;
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
    public double getCSArmPos180Deg(){
        return CS_ARM_POS_180_DEG;
    }
    public double getCSClawPosOpen(){
        return  CS_CLAW_POS_OPEN;
    }
    public double getCSClawPosClosed(){
        return CS_CLAW_POS_CLOSED;
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
        return ((angle / 180.0) * (this.getClawArmPos180Deg() - this.getClawArmPos0Deg())) + this.getClawArmPos0Deg();
    }
    public void setMainClawRotationDegrees(double angle) {
        mainClawArm.setPosition(this.mainClawRotationAngleToPos(angle));
    }
    public double mainClawRotationAngleToPos(double angle){
        return ((angle / 180.0) * (this.getClawArmRot180Deg() - this.getClawArmRot0Deg())) + this.getClawArmRot0Deg();
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
        return ((angle / 180.0) * (this.getCSArmPos180Deg() - this.getCSArmPos0Deg())) + this.getCSArmPos0Deg();
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
        int i = 0;
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
        int i = 0;
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
