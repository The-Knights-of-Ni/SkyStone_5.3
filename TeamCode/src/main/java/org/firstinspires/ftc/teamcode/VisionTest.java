package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;

/**
 * Created by tarunsingh on 12/5/17.
 * Modified by AndrewC on 12/27/2019.
 */

@TeleOp(name="Vision Test")
public class VisionTest extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    double mainArmHorizontalPos = 0.0;
    double mainArmVerticalPos = 0.0;
    double mainArmHorizontalMax = 1000.0;
    double mainArmVerticalMax = 1200.0;
    double mainArmIncrement = 600.0;
    double mainClawRotationAngle;
    double mainClawRotationIncrement = 300;
    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;

    private boolean mainClawArmControlDigital = true;
    private boolean mainClawArmDeployed = false;
    private boolean csClawArmControlDigital = true;
    private boolean csClawArmDeployed = false;

    // cameraSelection : 0 (front webcam), 1 (arm webcam)
    private int cameraSelection = 0;
    private int markerUpperLeftx;
    private int markerUpperLefty;
    private int markerLowerRightx;
    private int markerLowerRighty;
    private int markerCounter = 0;

    private void initOpMode(){
        telemetry.addData("Init Robot", "");
        telemetry.update();
        timer = new ElapsedTime();
        // visionMode 3: backWebcam is initialized for Vuforia and frontWebcam is initialized for OpenCV
        try {
            this.robot = new Robot(this, timer, 3);
        } catch (IOException e) {
            e.printStackTrace();
        }
        cameraSelection = 0;
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }
    public void runOpMode() {
        initOpMode();
        // call initServosTeleop() after running Auto program
//        robot.initServosTeleop();
        // call initServosAuto() if testing Teleop stand-alone
        robot.initServosAuto();
        waitForStart();

        mainClawRotationAngle = robot.control.getMainClawRotationDegrees();
        telemetry.clearAll();

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        robot.vision.getTargetsSkyStone().activate();

        mainArmHorizontalPos = 40.0;
        mainArmVerticalPos = 80.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());

        while (!isStopRequested()) {

            // Get gamepad inputs
            robot.getGamePadInputs();

            // Get the current time
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            robot.vision.vuMarkScan();

            // robot control
            // move foundation claw
            if (robot.bumperLeft && (!robot.bumperRight)) { // foundation claw up
                robot.control.raiseClawsFromFoundation();
            } else if (robot.bumperRight && (!robot.bumperLeft)) { // foundation claw down
                robot.control.lowerClawsToFoundation();
            }
            if ((robot.triggerLeft > 0.5) && (robot.triggerRight < 0.5)) { // foundation claw up
                robot.control.raiseClawsFromFoundation();
            } else if ((robot.triggerRight > 0.5) && (robot.triggerLeft < 0.5)) { // foundation claw down
                robot.control.lowerClawsToFoundation();
            }

            // deploy main claw arm
            if (mainClawArmControlDigital) {
                if (robot.bumperRight2 && !robot.isrBumper2PressedPrev) { // toggle main claw arm deploy mode
                    if (mainClawArmDeployed) {
                        robot.control.retractMainClawArm();
                        mainClawArmDeployed = false;
                    }
                    else {
                        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
                        mainClawArmDeployed = true;
                    }
                }
            }

            // deploy capstone claw arm
            if (csClawArmControlDigital) {
                if (robot.bumperLeft2 && !robot.islBumper2PressedPrev) { // toggle capstone claw arm deploy mode
                    if (csClawArmDeployed) {
                        robot.control.retractCSClawArm();
                        csClawArmDeployed = false;
                    }
                    else {
                        robot.control.setCSClawArmDegrees(robot.control.getMainArmTargetAngle());
                        csClawArmDeployed = true;
                    }
                }
            }

            // control main claw
            if ((robot.triggerLeft2 > 0.5) && (robot.triggerRight2 < 0.5)) { // main claw open
                robot.control.openMainClaw();
            } else if ((robot.triggerRight2 > 0.5) && (robot.triggerLeft2 < 0.5)) { // main claw close
                robot.control.closeMainClawStone();
            }

            // control capstone claw
            if (robot.yButton2 && (!robot.aButton2)) { // capstone claw open
                robot.control.openCSClaw();
            } else if (robot.aButton2 && (!robot.yButton2)) { // capstone claw close
                robot.control.closeCSClaw();
            }

            // move robot main arm
            // move robot main arm along horizontal line
            // move robot main arm
            // move robot main arm along horizontal line
            if(robot.leftStickY2 >= 0.1){
                mainArmHorizontalPos = mainArmHorizontalPos + (robot.leftStickY2 - 0.1) * (robot.leftStickY2 - 0.1) * mainArmIncrement * deltaT/1e9;
            }
            else if(robot.leftStickY2  <= -0.1){
                mainArmHorizontalPos = mainArmHorizontalPos - (robot.leftStickY2 + 0.1) * (robot.leftStickY2 + 0.1) * mainArmIncrement * deltaT/1e9;
            }
            if (mainArmHorizontalPos > mainArmHorizontalMax) {
                mainArmHorizontalPos = mainArmHorizontalMax;
            }
            if (mainArmHorizontalPos < 0.0) {
                mainArmHorizontalPos =0.0;
            }
            // move robot main arm along vertical line
            if(robot.rightStickY2 >= 0.1){
                mainArmVerticalPos = mainArmVerticalPos + (robot.rightStickY2 - 0.1) * (robot.rightStickY2 - 0.1) * mainArmIncrement * deltaT/1e9;
            }
            else if(robot.rightStickY2  <= -0.1){
                mainArmVerticalPos = mainArmVerticalPos - (robot.rightStickY2 + 0.1) * (robot.rightStickY2 + 0.1) * mainArmIncrement * deltaT/1e9;
            }
            if (mainArmVerticalPos > mainArmVerticalMax) {
                mainArmVerticalPos = mainArmVerticalMax;
            }
            if (mainArmVerticalPos < 0.0) {
                mainArmVerticalPos =0.0;
            }
            robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);

            // rotate main claw
            if(robot.rightStickX2 >= 0.1){
                mainClawRotationAngle = mainClawRotationAngle + (robot.rightStickX2 - 0.1) * (robot.rightStickX2 - 0.1) * mainClawRotationIncrement * deltaT/1e9;
            }
            else if(robot.rightStickX2  <= -0.1){
                mainClawRotationAngle = mainClawRotationAngle - (robot.rightStickX2 + 0.1) * (robot.rightStickX2 + 0.1) * mainClawRotationIncrement * deltaT/1e9;
            }
            if (mainClawRotationAngle > 180.0) {
                mainClawRotationAngle = 180.0;
            }
            if (mainClawRotationAngle < 0.0) {
                mainClawRotationAngle =0.0;
            }
            robot.control.setMainClawRotationDegrees(mainClawRotationAngle);

            if(mainClawArmControlDigital && mainClawArmDeployed){
                robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
            }
            if(csClawArmDeployed && csClawArmControlDigital){
                robot.control.setCSClawArmDegrees(robot.control.getMainArmTargetAngle());
            }

            //Automate skybrige pos
            if(robot.bButton2 && !robot.isbButton2PressedPrev){
                mainArmHorizontalPos = 40.0;
                mainArmVerticalPos = 80.0;
                robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
            }

            telemetry.addData("X Y Rot ", "%.1f,  %.1f,  %.1f", mainArmHorizontalPos, mainArmVerticalPos, mainClawRotationAngle);


            robot.getOpmode().telemetry.addData("Threshold", "{r, b, y1, y2} = %d, %d, %d, %d",
                    robot.vision.getRedColorThreshold(),robot.vision.getBlueColorThreshold(),
                    robot.vision.getYellowColorThreshold1(), robot.vision.getYellowColorThreshold2());

           // toggle cameras
            if (robot.bButton && !robot.isbButtonPressedPrev) {
                if (cameraSelection == 0) {
                    // change from front webcam to arm webcam
                    robot.vision.closeFrontWebcam();
                    sleep(200);
                    robot.vision.initArmWebcam(1);
                    cameraSelection = 1;
                }
                else {
                    // change from arm webcam to front webcam
                    robot.vision.closeArmWebcam();
                    sleep(200);
                    robot.vision.initFrontWebcam(1);
                    cameraSelection = 0;
                }
            }
            if (cameraSelection == 0) {
                robot.getOpmode().telemetry.addData("camera selected (b): ", "front webcam");
            }
            else {
                robot.getOpmode().telemetry.addData("camera selected (b): ", "arm webcam");
            }

            // toggle tapping mode
            if (robot.xButton && !robot.isxButtonPressedPrev) {
                int currentTappingModeNum = robot.vision.getTappingMode().ordinal();
                int nextTappingModeNum = currentTappingModeNum + 1;
                if (nextTappingModeNum >= robot.vision.tappingModes.length) {
                    nextTappingModeNum = 0;
                }
                robot.vision.setTappingMode(robot.vision.tappingModes[nextTappingModeNum]);
            }
//            robot.getOpmode().telemetry.addData("Tapping mode (x): ", robot.vision.getTappingMode().toString());

            // toggle detected color
            if (robot.yButton && !robot.isyButtonPressedPrev) {
                int currentDetectedColorNum = robot.vision.getDetectedColor().ordinal();
                int nextDetectedColorNum = currentDetectedColorNum + 1;
                if (nextDetectedColorNum >= robot.vision.detectedColors.length) {
                    nextDetectedColorNum = 0;
                }
                robot.vision.setDetectedColor(robot.vision.detectedColors[nextDetectedColorNum]);
            }
//            robot.getOpmode().telemetry.addData("Detected color (y): ", robot.vision.getDetectedColor().toString());
            robot.getOpmode().telemetry.addData("Tapping (x): ","%s  Color (y): %s",
                    robot.vision.getTappingMode().toString(), robot.vision.getDetectedColor().toString());

//            robot.getOpmode().telemetry.addData("Contours found: ", robot.vision.getNumContoursFound());

            // marker window corners
            int[] corners = robot.vision.getMarkerCorners();
            markerUpperLeftx = corners[0];
            markerUpperLefty = corners[1];
            markerLowerRightx = corners[2];
            markerLowerRighty = corners[3];
            if (markerCounter == 0) {
                if (robot.rightStickX > 0.2) markerLowerRightx = markerLowerRightx + 1;
                if (robot.rightStickX < -0.2) markerLowerRightx = markerLowerRightx - 1;
                if (robot.rightStickY > 0.2) markerLowerRighty = markerLowerRighty + 1;
                if (robot.rightStickY < -0.2) markerLowerRighty = markerLowerRighty - 1;
                if (robot.leftStickX > 0.2) markerUpperLeftx = markerUpperLeftx + 1;
                if (robot.leftStickX < -0.2) markerUpperLeftx = markerUpperLeftx - 1;
                if (robot.leftStickY > 0.2) markerUpperLefty = markerUpperLefty - 1;
                if (robot.leftStickY < -0.2) markerUpperLefty = markerUpperLefty + 1;
            }
            markerCounter = markerCounter + 1;
            if (markerCounter == 10) markerCounter = 0;
            corners[0] = markerUpperLeftx;
            corners[1] = markerUpperLefty;
            corners[2] = markerLowerRightx;
            corners[3] = markerLowerRighty;
            robot.vision.setMarkerCorners(corners);
            robot.getOpmode().telemetry.addData("Marker: ", "%d, %d, %d, %d",
                    markerUpperLeftx, markerUpperLefty, markerLowerRightx, markerLowerRighty);

            // save images
            if (robot.aButton && !robot.isaButtonPressedPrev) {
                robot.getOpmode().telemetry.addData("saving ", "images...");
                robot.vision.saveImage("VisionTest", robot.vision.frameBuffer2, Imgproc.COLOR_RGBA2BGR, "original", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.frameBuffer1, Imgproc.COLOR_RGBA2BGR, "undistorted", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan2Mat_compensatedn, Imgproc.COLOR_RGBA2BGR, "CbImage_c25n", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan2Mat_compensated, Imgproc.COLOR_RGBA2BGR, "CbImage_c25", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan1Mat_compensated, Imgproc.COLOR_RGBA2BGR, "CrImage_c25", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan2Mat, Imgproc.COLOR_RGBA2BGR, "CbImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan1Mat, Imgproc.COLOR_RGBA2BGR, "CrImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan0Mat, Imgproc.COLOR_RGBA2BGR, "YImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.thresholdMat, Imgproc.COLOR_RGBA2BGR, "threshold", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.contoursOnFrameMat, Imgproc.COLOR_RGBA2BGR, "contours", (long) timeCurrent);
            }

            robot.getOpmode().telemetry.update();

        }

        // Disable Tracking when we are done;
        robot.vision.getTargetsSkyStone().deactivate();

    }
}