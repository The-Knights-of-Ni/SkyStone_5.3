package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.mean;

/**
 * SkyStone Auto Mode for Blue Alliance
 * Created by Andrew Chiang on 1/20/2020
 */
@Autonomous(name = "Auto_Blue")
public class Auto_Blue extends LinearOpMode {
    private Robot robot;

    ElapsedTime timer;
    double timeCurrent;
    double timeStart;
    double timePre;
    double deltaT;

    double mainArmHorizontalPos = 0.0;
    double mainArmVerticalPos = 0.0;
    double mainArmHorizontalMax = 1000.0;
    double mainArmVerticalMax = 1200.0;
    double mainClawRotationAngle;

    double robotAngle;

    enum Alliance {
        BLUE,
        RED,
    }
    Alliance alliance = Alliance.BLUE;

    // visual markers for aligning stones to robot initial position
    private int[] initMarkerCornersBlue = {51, 120, 282, 159};    // Blue Alliance
    private int[] initMarkerCornersRed = {36, 125, 273, 162};    // Red Alliance

    private int[] block1CornersBlue = {70, 129, 107, 150};      // stone {51, 120, 126, 159}
    private int[] block2CornersBlue = {145, 129, 184, 150};     // stone {126, 120, 203, 159}
    private int[] block3CornersBlue = {223, 129, 263, 150};     // stone {203, 120, 282, 159}
    private int[] block1CornersRed = {56, 134, 96, 153};        // stone {36, 125, 115, 162}
    private int[] block2CornersRed = {135, 134, 174, 153};      // stone {115, 125, 193, 162}
    private int[] block3CornersRed = {213, 134, 253, 153};      // stone {193, 125, 273, 162}

    enum SkyStonePattern {
        PATTERNA,
        PATTERNB,
        PATTERNC,
    }
    private SkyStonePattern skyStonePattern;
    double[] stoneOffset;

    private void initOpMode() throws IOException {
        telemetry.addData("Init Robot", "");
        telemetry.update();
        timer = new ElapsedTime();
        // visionMode 3: backWebcam is initialized for Vuforia and frontWebcam is initialized for OpenCV
        this.robot = new Robot(this, timer, 3);
        // define visual marker corners
        if (alliance == Alliance.BLUE) {
            robot.vision.setMarkerCorners(initMarkerCornersBlue);
        }
        else {
            robot.vision.setMarkerCorners(initMarkerCornersRed);
        }
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        robot.initServosAuto();
        waitForStart();

        timeCurrent = timer.nanoseconds();
        timeStart = timeCurrent;
        timePre = timeCurrent;
        telemetry.clearAll();

        // rotate main claw to open sideways
        mainClawRotationAngle = 0.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);

        // activate Vuforia
        robot.vision.getTargetsSkyStone().activate();

        // use the front camera image to determine the SkyStone pattern
        skyStonePattern = findSkyStoneLocation();

        // deploy main claw arm
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());

        // switch camera to the arm camera
        robot.vision.closeFrontWebcam();
        robot.vision.initArmWebcam(1);

        // setup main claw and main arm initial position
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        mainClawRotationAngle = 0.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);
        robot.control.openMainClawWide();
        mainArmHorizontalPos = 40.0;
        mainArmVerticalPos = 80.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(800);

        // move out from the wall
        robot.drive.moveForward(570);
//        sleep(100);
//        printRobotPosition();
        // detect Skystone offset from the arm camera
        sleep(100);
        stoneOffset = getStoneOffset();
        saveImages();
//        sleep(3000);

        // move to the left depending on the SkyStone pattern
        switch (skyStonePattern) {
            case PATTERNA:
//                robot.drive.setDriveFullPower(true);
//                robot.drive.moveLeft(20);
//                robot.drive.setDriveFullPower(false);
                break;
            case PATTERNB:
                robot.drive.moveLeft(203);
                break;
            case PATTERNC:
                robot.drive.moveLeft(406);
                break;
            default:
                break;
        }
        // detect Skystone offset from the arm camera
        sleep(300);
        stoneOffset = getStoneOffset();
        saveImages();
//        sleep(3000);

        // rotate main claw
        mainClawRotationAngle = 90.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);

        pickupSkySTone(stoneOffset[1]);
//        sleep(100);
//        printRobotPosition();
        sleep(500);
        robot.drive.turnRobotByTick(90.0);
//        robot.drive.turnRobot(90.0);

//        printRobotPosition();
        sleep(100);

        // Move to different position depending on where skystone is detected
        switch (skyStonePattern) {
            case PATTERNA:
                robot.drive.moveForward(2236-50);
                break;
            case PATTERNB:
                robot.drive.moveForward(2033-50);
                break;
            case PATTERNC:
                robot.drive.moveForward(1830-50);
                break;
            default:
                break;
        }
//        sleep(5000);
//        printRobotPosition();
        sleep(100);

        mainArmHorizontalPos = 0.0;
        mainArmVerticalPos = 120.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());

        robot.drive.turnRobotByTick(-90.0);
//        robot.drive.turnRobot(90.0);

//        sleep(5000);
//        printRobotPosition();
        sleep(100);
        stoneOffset = getBlueFoundationOffset();
        saveImages();
//        sleep(3000);

        // rotate main claw
        mainClawRotationAngle = 0.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);

        mainArmHorizontalPos = 90.0 + 30.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        robot.drive.moveForward(150 + stoneOffset[1]);
        sleep(200);
//        saveImages();
//        sleep(3000);

        mainArmVerticalPos = 70.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(500);
        robot.control.openMainClawWide();
        sleep(300);

        mainArmHorizontalPos = 90.0;
        mainArmVerticalPos = 120.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(300);
        mainArmHorizontalPos = 0.0;
        mainArmVerticalPos = 120.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());

//        robot.drive.moveRight(30);
        robot.drive.moveLeft(20);
        sleep(100);

        robot.control.lowerClawsToFoundation();
        mainClawRotationAngle = 0.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);
//        robot.control.closeMainClawStone();
        sleep(600);


        pullbackBlueFoundation();
//        sleep(100);

        pushBlueFoundation();
//        sleep(100);

        robot.control.raiseClawsFromFoundation();
        sleep(300);
        robot.drive.moveBackward(400);

        mainArmHorizontalPos = 0.0;
        mainArmVerticalPos = 0.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
//        robot.control.retractMainClawArm();
//        sleep(1000);
        sleep(100);
        robot.control.closeMainClawStone();

        robot.drive.turnRobotByTick(10.0);

        parkRobot();

        // Disable Tracking when we are done;
        robot.vision.getTargetsSkyStone().deactivate();
        if (robot.vision.frontWebcamIsActive) {
            robot.vision.closeFrontWebcam();
        }
        if (robot.vision.armWebcamIsActive) {
            robot.vision.closeArmWebcam();
        }
    }

    private void pullbackBlueFoundation() {
        double initialAngle = robot.drive.getYaw();
        double currentAngle;
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.frontLeft.setTargetPosition(0);
        robot.drive.frontRight.setTargetPosition(0);
        robot.drive.rearLeft.setTargetPosition(0);
        robot.drive.rearRight.setTargetPosition(0);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        double motorPowerLeft = 0.3;
        double motorPowerRight = 0.3;
        robot.drive.rearLeft.setPower(motorPowerLeft);
        robot.drive.frontLeft.setPower(motorPowerLeft);
        robot.drive.rearRight.setPower(motorPowerRight);
        robot.drive.frontRight.setPower(motorPowerRight);

        // start moving backward
        robot.drive.frontLeft.setTargetPosition(-5000);
        robot.drive.frontRight.setTargetPosition(-5000);
        robot.drive.rearLeft.setTargetPosition(-5000);
        robot.drive.rearRight.setTargetPosition(-5000);

        while (robot.drive.rearLeft.getCurrentPosition() > -200) {

        }
        motorPowerLeft = 0.6;
        motorPowerRight = 0.1;
        robot.drive.rearLeft.setPower(motorPowerLeft);
        robot.drive.frontLeft.setPower(motorPowerLeft);
        robot.drive.rearRight.setPower(motorPowerRight);
        robot.drive.frontRight.setPower(motorPowerRight);

        while (robot.drive.rearLeft.getCurrentPosition() > -1100) {

        }

        motorPowerRight = 0.6;
        robot.drive.rearRight.setPower(motorPowerRight);
        robot.drive.frontRight.setPower(motorPowerRight);
        robot.drive.frontRight.setTargetPosition(4000);
        robot.drive.rearRight.setTargetPosition(4000);

        // keep going until robot has finished turning
        boolean keepGoing = true;
        while (keepGoing) {
            currentAngle = robot.drive.getYaw();
            if (((currentAngle - initialAngle > 80.0) && (currentAngle - initialAngle < 95.0)) ||
                    ((currentAngle - initialAngle + 360.0 > 80.0) && (currentAngle - initialAngle + 360.0 < 95.0))) {
                keepGoing = false;
            }
        }

        robot.drive.stop();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.frontLeft.setTargetPosition(0);
        robot.drive.frontRight.setTargetPosition(0);
        robot.drive.rearLeft.setTargetPosition(0);
        robot.drive.rearRight.setTargetPosition(0);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.stop();

    }

    private void pushBlueFoundation() {
        robot.drive.moveForward(500, 0.2);
        robot.drive.stop();
    }

    private void parkRobot() {
        robot.drive.moveBackward(600, 0.15);
        robot.drive.stop();
    }

    /**
     * Use relative Cb strength in 3 locations on the camera image to determine the SkyStone location
     * @return PATTERNA : Pattern A (SkyStone is closest to the wall)
     *         PATTERNB : Pattern B (SkyStone is in the middle)
     *         PATTERNC : Pattern C (SkyStone is closest to the bridge)
     */
    private SkyStonePattern findSkyStoneLocation() {
        Mat block1, block2, block3;
        Scalar mean1, mean2, mean3, mean1x, mean2x, mean3x;
        SkyStonePattern skyStonePattern = SkyStonePattern.PATTERNC;
        if (alliance == Alliance.BLUE) {
            // use a fixed threshold to detect the Skystone
//            block1 = robot.vision.thresholdMat.submat(block1CornersBlue[1], block1CornersBlue[3], block1CornersBlue[0], block1CornersBlue[2]);
//            block2 = robot.vision.thresholdMat.submat(block2CornersBlue[1], block2CornersBlue[3], block2CornersBlue[0], block2CornersBlue[2]);
//            block3 = robot.vision.thresholdMat.submat(block3CornersBlue[1], block3CornersBlue[3], block3CornersBlue[0], block3CornersBlue[2]);
//            mean1 = Core.mean(block1);
//            mean2 = Core.mean(block2);
//            mean3 = Core.mean(block3);

            // use analog Cb image to detect the Skystone
            block1 = robot.vision.yCbCrChan2Mat.submat(block1CornersBlue[1], block1CornersBlue[3], block1CornersBlue[0], block1CornersBlue[2]);
            block2 = robot.vision.yCbCrChan2Mat.submat(block2CornersBlue[1], block2CornersBlue[3], block2CornersBlue[0], block2CornersBlue[2]);
            block3 = robot.vision.yCbCrChan2Mat.submat(block3CornersBlue[1], block3CornersBlue[3], block3CornersBlue[0], block3CornersBlue[2]);
            mean1x = Core.mean(block1);
            mean2x = Core.mean(block2);
            mean3x = Core.mean(block3);
            // Core.mean() returns a Scalar which is a vector of four doubles
            // yCbCrChan2Mat was the extracted Cb plane, there is only one mean value in the returned Scalar (the rest three values are zeros)
            // yellow stone has lower Cb values than the SkyStone which is essentially black
            // on the Blue Alliance side, the camera sees the 3 stones closest to the bridge
            if ((mean1x.val[0] > mean2x.val[0]) && (mean1x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.PATTERNC;
            if ((mean2x.val[0] > mean1x.val[0]) && (mean2x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.PATTERNB;
            if ((mean3x.val[0] > mean1x.val[0]) && (mean3x.val[0] > mean2x.val[0])) skyStonePattern = SkyStonePattern.PATTERNA;
        }
        else {  // Red alliance
            // use a fixed threshold to detect the Skystone
//            block1 = robot.vision.thresholdMat.submat(block1CornersRed[1], block1CornersRed[3], block1CornersRed[0], block1CornersRed[2]);
//            block2 = robot.vision.thresholdMat.submat(block2CornersRed[1], block2CornersRed[3], block2CornersRed[0], block2CornersRed[2]);
//            block3 = robot.vision.thresholdMat.submat(block3CornersRed[1], block3CornersRed[3], block3CornersRed[0], block3CornersRed[2]);
//            mean1 = Core.mean(block1);
//            mean2 = Core.mean(block2);
//            mean3 = Core.mean(block3);

            // use analog Cb image to detect the Skystone
            block1 = robot.vision.yCbCrChan2Mat.submat(block1CornersRed[1], block1CornersRed[3], block1CornersRed[0], block1CornersRed[2]);
            block2 = robot.vision.yCbCrChan2Mat.submat(block2CornersRed[1], block2CornersRed[3], block2CornersRed[0], block2CornersRed[2]);
            block3 = robot.vision.yCbCrChan2Mat.submat(block3CornersRed[1], block3CornersRed[3], block3CornersRed[0], block3CornersRed[2]);
            mean1x = Core.mean(block1);
            mean2x = Core.mean(block2);
            mean3x = Core.mean(block3);
            // Core.mean() returns a Scalar which is a vector of four doubles
            // on the Red Alliance side, the camera sees the 3 stones next to the one closest to the bridge
            if ((mean1x.val[0] > mean2x.val[0]) && (mean1x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.PATTERNC;
            if ((mean2x.val[0] > mean1x.val[0]) && (mean2x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.PATTERNA;
            if ((mean3x.val[0] > mean1x.val[0]) && (mean3x.val[0] > mean2x.val[0])) skyStonePattern = SkyStonePattern.PATTERNB;
        }

//        telemetry.addData("block1 mean ", "%.2f,   %.2f", mean1.val[0], mean1x.val[0]);
//        telemetry.addData("block2 mean ", "%.2f,   %.2f", mean2.val[0], mean2x.val[0]);
//        telemetry.addData("block3 mean ", "%.2f,   %.2f", mean3.val[0], mean3x.val[0]);
        telemetry.addData("block1 mean ", "%.2f", mean1x.val[0]);
        telemetry.addData("block2 mean ", "%.2f", mean2x.val[0]);
        telemetry.addData("block3 mean ", "%.2f", mean3x.val[0]);
        telemetry.addData("SkyStone pattern ", skyStonePattern.toString());
        telemetry.update();
        String output = String.format("Skystone Side: block1 mean %.2f, block2 mean %.2f, block3 mean %.2f, SkyStone pattern %s",
                mean1x.val[0], mean2x.val[0], mean3x.val[0], skyStonePattern.toString());
        Log.d("autoVision", output);

        return skyStonePattern;
    }

    /**
     * use arm camera image to detect the offset of Skystone
     * @return  xOffset: lateral offset of the Skystone
     *          yOffset: longitudinal offset of the Skystone
     *          angle: angular offset of the Skystone
     */
    private double[] getStoneOffset() {
        double xOffset = 0.0;
        double yOffset = 0.0;
        double angle = 0.0;
        double maxLevel, minLevel, thresholdLevel;
        Mat block1, block2, block3, block4, block5;
        Scalar mean1, mean2, mean3, mean4, mean5;
        Mat thresholdMat = new Mat();
        // use analog Cb image to detect the Skystone
        // yCbCrChan2Mat_compensatedn is negatively compensated by Y image to deal with the region with strong reflection
        // yellow stone region should have low Cb value, but the strong reflection is white which will have mid-level Cb value
        // yCbCrChan2Mat_compensatedn is 0.75*Cb - 0.25*Y + 64
        block1 = robot.vision.yCbCrChan2Mat_compensatedn.submat(1, 21, 150, 170);
        block2 = robot.vision.yCbCrChan2Mat_compensatedn.submat(55, 75, 150, 170);
        block3 = robot.vision.yCbCrChan2Mat_compensatedn.submat(110, 130, 150, 170);
        block4 = robot.vision.yCbCrChan2Mat_compensatedn.submat(165, 185, 150, 170);
        block5 = robot.vision.yCbCrChan2Mat_compensatedn.submat(218, 238, 150, 170);
        mean1 = Core.mean(block1);
        mean2 = Core.mean(block2);
        mean3 = Core.mean(block3);
        mean4 = Core.mean(block4);
        mean5 = Core.mean(block5);
        telemetry.addData("block1 mean ", "%.2f", mean1.val[0]);
        telemetry.addData("block2 mean ", "%.2f", mean2.val[0]);
        telemetry.addData("block3 mean ", "%.2f", mean3.val[0]);
        telemetry.addData("block4 mean ", "%.2f", mean4.val[0]);
        telemetry.addData("block5 mean ", "%.2f", mean5.val[0]);
        telemetry.update();
        maxLevel = Math.max(mean1.val[0], mean2.val[0]);
        maxLevel = Math.max(maxLevel, mean3.val[0]);
        maxLevel = Math.max(maxLevel, mean4.val[0]);
        maxLevel = Math.max(maxLevel, mean5.val[0]);
        minLevel = Math.min(mean1.val[0], mean2.val[0]);
        minLevel = Math.min(minLevel, mean3.val[0]);
        minLevel = Math.min(minLevel, mean4.val[0]);
        minLevel = Math.min(minLevel, mean5.val[0]);
        thresholdLevel = (minLevel + maxLevel) * 0.5;
        String output = String.format("Skystone Top: block1 %.2f, block2 %.2f, block3 %.2f, block4 %.2f, block5 %.2f, threshold %.2f",
                mean1.val[0], mean2.val[0], mean3.val[0], mean4.val[0], mean5.val[0], thresholdLevel);
        Log.d("autoVision", output);
        Imgproc.threshold(robot.vision.yCbCrChan2Mat_compensatedn, thresholdMat, (int) thresholdLevel, 255, Imgproc.THRESH_BINARY_INV);
        timeCurrent = timer.nanoseconds();
        robot.vision.saveImage("autoVision", thresholdMat, Imgproc.COLOR_RGBA2BGR, "StoneThreshold", (long) timeCurrent);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.erode(thresholdMat, thresholdMat, kernel);
        robot.vision.saveImage("autoVision", thresholdMat, Imgproc.COLOR_RGBA2BGR, "erode", (long) timeCurrent);
        Imgproc.dilate(thresholdMat, thresholdMat, kernel);
        robot.vision.saveImage("autoVision", thresholdMat, Imgproc.COLOR_RGBA2BGR, "erodedilate", (long) timeCurrent);

        // extract data array from Mat
        byte[] data = new byte[thresholdMat.cols()*thresholdMat.rows()];
        thresholdMat.get(0, 0, data);
        int minY = -1;
        int maxY = -1;
        for (int y = 0; y < thresholdMat.rows(); ++y) {
            if ((data[y*thresholdMat.cols() + thresholdMat.cols()/2] != 0) && (minY == -1)) {
                minY = y;
            }
            if ((data[y*thresholdMat.cols() + thresholdMat.cols()/2] == 0) && (maxY == -1) && (minY != -1)) {
                maxY = y;
            }
        }
        yOffset = (double) (70 - minY)*0.9;

        output = String.format("Skystone Top: minY %d, maxY %d, yOffset %.1f", minY, maxY, yOffset);
        Log.d("autoVision", output);

        return new double[] {xOffset, yOffset, angle};
    }

    /**
     * use arm camera image to detect the offset of Blue Foundation
     * @return  xOffset: lateral offset of the Foundation
     *          yOffset: longitudinal offset of the Foundation
     *          angle: angular offset of the Foundation
     */
    private double[] getBlueFoundationOffset() {
        double xOffset = 0.0;
        double yOffset = 0.0;
        double angle = 0.0;
        double maxLevel, minLevel, thresholdLevel;
        Mat block1, block2, block3, block4, block5;
        Scalar mean1, mean2, mean3, mean4, mean5;
        Mat thresholdMat = new Mat();
        // use analog Cb image to detect the Blue Foundation
        // yCbCrChan2Mat_compensated is positively compensated by Y image to deal with the region with strong reflection
        // Blue Foundation region should have high Cb value, but the strong reflection is white which will have mid-level Cb value
        // yCbCrChan2Mat_compensated is 0.75*Cb + 0.25*Y
        block1 = robot.vision.yCbCrChan2Mat_compensated.submat(1, 21, 50, 70);
        block2 = robot.vision.yCbCrChan2Mat_compensated.submat(30, 50, 150, 170);
        block3 = robot.vision.yCbCrChan2Mat_compensated.submat(1, 21, 250, 270);
        block4 = robot.vision.yCbCrChan2Mat_compensated.submat(200, 220, 5, 25);
        block5 = robot.vision.yCbCrChan2Mat_compensated.submat(200, 220, 295, 315);
        mean1 = Core.mean(block1);
        mean2 = Core.mean(block2);
        mean3 = Core.mean(block3);
        mean4 = Core.mean(block4);
        mean5 = Core.mean(block5);
        telemetry.addData("block1 mean ", "%.2f", mean1.val[0]);
        telemetry.addData("block2 mean ", "%.2f", mean2.val[0]);
        telemetry.addData("block3 mean ", "%.2f", mean3.val[0]);
        telemetry.addData("block4 mean ", "%.2f", mean4.val[0]);
        telemetry.addData("block5 mean ", "%.2f", mean5.val[0]);
        telemetry.update();
        maxLevel = Math.max(mean1.val[0], mean2.val[0]);
        maxLevel = Math.max(maxLevel, mean3.val[0]);        // find the max level in the foundation region
        minLevel = Math.max(mean4.val[0], mean5.val[0]);    // depending on the position of the Skystone, one of these might not be floor
                                                            // the stone has lower Cb level than the floor
        thresholdLevel = minLevel * 0.55 + maxLevel * 0.45;
        String output = String.format("Blue Foundation: block1 %.2f, block2 %.2f, block3 %.2f, block4 %.2f, block5 %.2f, threshold %.2f",
                mean1.val[0], mean2.val[0], mean3.val[0], mean4.val[0], mean5.val[0], thresholdLevel);
        Log.d("autoVision", output);
        Imgproc.threshold(robot.vision.yCbCrChan2Mat_compensated, thresholdMat, (int) thresholdLevel, 255, Imgproc.THRESH_BINARY);
        timeCurrent = timer.nanoseconds();
        robot.vision.saveImage("autoVision", thresholdMat, Imgproc.COLOR_RGBA2BGR, "BlueFThreshold", (long) timeCurrent);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2*2) + 1, (2*2)+1));
        Imgproc.dilate(thresholdMat, thresholdMat, kernel);
        robot.vision.saveImage("autoVision", thresholdMat, Imgproc.COLOR_RGBA2BGR, "dilate", (long) timeCurrent);
        Imgproc.erode(thresholdMat, thresholdMat, kernel);
        robot.vision.saveImage("autoVision", thresholdMat, Imgproc.COLOR_RGBA2BGR, "dilateerode", (long) timeCurrent);

        // extract data array from Mat
        byte[] data = new byte[thresholdMat.cols()*thresholdMat.rows()];
        thresholdMat.get(0, 0, data);
        int maxY0 = -1;
        int maxY1 = -1;
        int maxY2 = -1;
        for (int y = 1; y < thresholdMat.rows(); ++y) {
            if ((data[y*thresholdMat.cols() + thresholdMat.cols()/2 - 50] == 0) && (maxY0 == -1)) {
                maxY0 = y;
            }
            if ((data[y*thresholdMat.cols() + thresholdMat.cols()/2     ] == 0) && (maxY1 == -1)) {
                maxY1 = y;
            }
            if ((data[y*thresholdMat.cols() + thresholdMat.cols()/2 + 50] == 0) && (maxY2 == -1)) {
                maxY2 = y;
            }
        }
        int maxY;
        maxY = Math.max(maxY0, maxY1);
        maxY = Math.max(maxY, maxY2);
        yOffset = (double) (126 - maxY)*1.513;

        output = String.format("Blue Foundation: maxY0 %d, maxY1 %d, maxY2 %d, yOffset %.1f", maxY0, maxY1, maxY2, yOffset);
        Log.d("autoVision", output);

        return new double[] {xOffset, yOffset, angle};
    }

    private void pickupSkySTone(double yOffset) {
        robot.control.openMainClawWide();

        // extend arm
        mainArmHorizontalPos = 139.0 + yOffset;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(500);

        // lower arm to get stone
        mainArmVerticalPos = 0.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(400);
        robot.control.closeMainClawStone();

        // raise arm
        sleep(500);
        mainArmVerticalPos = 50.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        sleep(300);

        // retract arm
        mainArmHorizontalPos = 0.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(400);
    }

    private void printRobotPosition() {
        sleep(3000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "1 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "2 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "3 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "4 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "5 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "6 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "7 sec");
//        telemetry.update();
//        sleep(1000);
//        robot.vision.vuMarkScan();
//        telemetry.addData("after ", "8 sec");
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        telemetry.addData("robot angle ", "%.1f", robot.drive.getYaw());
        sleep(1000);
        robotAngle = robot.drive.getYaw();
        telemetry.addData("robot angle ", "%.1f", robotAngle);
        telemetry.update();
    }

    private void saveImages() {
        timeCurrent = timer.nanoseconds();
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
}
