package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

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
        BATTERNC,
    }
    private SkyStonePattern skyStonePattern;

    private void initOpMode(){
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
        initOpMode();
        robot.initServosAuto();
        waitForStart();

        timeCurrent = timer.nanoseconds();
        timeStart = timeCurrent;
        timePre = timeCurrent;

        mainClawRotationAngle = robot.control.getMainClawRotationDegrees();
        telemetry.clearAll();

        robot.vision.getTargetsSkyStone().activate();

        // use the front camera image to determine the SkyStone pattern
        skyStonePattern = findSkyStoneLocation();


        // setup main arm and claw position
        mainClawRotationAngle = 90.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        mainArmHorizontalPos = 40.0;
        mainArmVerticalPos = 80.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());

        // move out from the wall
        robot.drive.moveForward(570);
        sleep(100);
        printRobotPosition();
        sleep(10000);

        // move to the left depending on the SkyStone pattern
        switch (skyStonePattern) {
            case PATTERNA:
//                robot.drive.setDriveFullPower(true);
//                robot.drive.moveLeft(20);
//                robot.drive.setDriveFullPower(false);
                break;
            case PATTERNB:
                robot.drive.moveLeft(223);
                break;
            case BATTERNC:
                robot.drive.moveLeft(426);
                break;
            default:
                break;
        }
        sleep(100);
        pickupSkySTone();
        sleep(100);
//        robot.drive.turnRobotByTick(90.0);
        robot.drive.turnRobot(90.0);

        printRobotPosition();
        sleep(10000);

        // Disable Tracking when we are done;
        robot.vision.getTargetsSkyStone().deactivate();
        robot.vision.closeFrontWebcam();
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
        SkyStonePattern skyStonePattern = SkyStonePattern.BATTERNC;
        if (alliance == Alliance.BLUE) {
//            block1 = robot.vision.thresholdMat.submat(block1CornersBlue[1], block1CornersBlue[3], block1CornersBlue[0], block1CornersBlue[2]);
//            block2 = robot.vision.thresholdMat.submat(block2CornersBlue[1], block2CornersBlue[3], block2CornersBlue[0], block2CornersBlue[2]);
//            block3 = robot.vision.thresholdMat.submat(block3CornersBlue[1], block3CornersBlue[3], block3CornersBlue[0], block3CornersBlue[2]);
//            mean1 = Core.mean(block1);
//            mean2 = Core.mean(block2);
//            mean3 = Core.mean(block3);

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
            if ((mean1x.val[0] > mean2x.val[0]) && (mean1x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.BATTERNC;
            if ((mean2x.val[0] > mean1x.val[0]) && (mean2x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.PATTERNB;
            if ((mean3x.val[0] > mean1x.val[0]) && (mean3x.val[0] > mean2x.val[0])) skyStonePattern = SkyStonePattern.PATTERNA;
        }
        else {  // Red alliance
//            block1 = robot.vision.thresholdMat.submat(block1CornersRed[1], block1CornersRed[3], block1CornersRed[0], block1CornersRed[2]);
//            block2 = robot.vision.thresholdMat.submat(block2CornersRed[1], block2CornersRed[3], block2CornersRed[0], block2CornersRed[2]);
//            block3 = robot.vision.thresholdMat.submat(block3CornersRed[1], block3CornersRed[3], block3CornersRed[0], block3CornersRed[2]);
//            mean1 = Core.mean(block1);
//            mean2 = Core.mean(block2);
//            mean3 = Core.mean(block3);

            block1 = robot.vision.yCbCrChan2Mat.submat(block1CornersRed[1], block1CornersRed[3], block1CornersRed[0], block1CornersRed[2]);
            block2 = robot.vision.yCbCrChan2Mat.submat(block2CornersRed[1], block2CornersRed[3], block2CornersRed[0], block2CornersRed[2]);
            block3 = robot.vision.yCbCrChan2Mat.submat(block3CornersRed[1], block3CornersRed[3], block3CornersRed[0], block3CornersRed[2]);
            mean1x = Core.mean(block1);
            mean2x = Core.mean(block2);
            mean3x = Core.mean(block3);
            // Core.mean() returns a Scalar which is a vector of four doubles
            // on the Red Alliance side, the camera sees the 3 stones next to the one closest to the bridge
            if ((mean1x.val[0] > mean2x.val[0]) && (mean1x.val[0] > mean3x.val[0])) skyStonePattern = SkyStonePattern.BATTERNC;
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

        return skyStonePattern;
    }

    private void pickupSkySTone() {
        robot.control.openMainClaw();
        mainArmHorizontalPos = 132.0;
        mainArmVerticalPos = 50.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(300);
        mainArmVerticalPos = 5.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
//        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(2000);
        robot.control.closeMainClawStone();
        sleep(1000);
        mainArmVerticalPos = 50.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        sleep(200);
        mainArmHorizontalPos = 0.0;
        robot.control.setMainArmPosition(mainArmHorizontalPos, mainArmVerticalPos);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(300);
    }

    private void printRobotPosition() {
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "1 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "2 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "3 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "4 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "5 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "6 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "7 sec");
        telemetry.update();
        sleep(1000);
        robot.vision.vuMarkScan();
        telemetry.addData("after ", "8 sec");
        robotAngle = robot.drive.getYaw();
        telemetry.addData("robot angle ", "%.1f", robotAngle);
        telemetry.update();
    }
}
