package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Robot;
import org.opencv.imgproc.Imgproc;

/**
 * Created by tarunsingh on 12/5/17.
 * Modified by AndrewC on 12/27/2019.
 */

@TeleOp(name="Vision Test")
public class VisionTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private static final float mmPerInch        = 25.4f;

    private Robot robot;

    double timeCurrent;

    // cameraSelection : 0 (front webcam), 1 (arm webcam)
    private int cameraSelection = 0;

    public void initOpMode(){
        ElapsedTime timer = new ElapsedTime();
        // visionMode 3: backWebcam is initialized for Vuforia and frontWebcam is initialized for OpenCV
        this.robot = new Robot(this, timer, 3);
        cameraSelection = 0;
    }
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        initOpMode();
        waitForStart();
        robot.vision.getTargetsSkyStone().activate();
        while (!isStopRequested()) {

            // Get gamepad inputs
            robot.getGamePadInputs();


            robot.vision.vuMarkScan();
            robot.getOpmode().telemetry.addData("Threshold", "{r, b, y1, y2} = %d, %d, %d, %d",
                    robot.vision.getRedColorThreshold(),robot.vision.getBlueColorThreshold(),
                    robot.vision.getYellowColorThreshold1(), robot.vision.getYellowColorThreshold2());

            timeCurrent = timer.nanoseconds();

            // save images
            if (robot.aButton2 && !robot.isaButton2PressedPrev) {
                robot.getOpmode().telemetry.addData("saving ", "images...");
                robot.vision.saveImage("VisionTest", robot.vision.frameBuffer1, Imgproc.COLOR_RGBA2BGR, "original", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan2Mat, Imgproc.COLOR_RGBA2BGR, "CbImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan1Mat, Imgproc.COLOR_RGBA2BGR, "CrImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.thresholdMat, Imgproc.COLOR_RGBA2BGR, "threshold", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.contoursOnFrameMat, Imgproc.COLOR_RGBA2BGR, "contours", (long) timeCurrent);
            }

            // toggle cameras
            if (robot.bButton2 && !robot.isbButton2PressedPrev) {
                if (cameraSelection == 0) {
                    robot.getOpmode().telemetry.addData("camera selected: ", "front webcam");
                }
                else {
                    robot.getOpmode().telemetry.addData("camera selected: ", "arm webcam");
                }
            }
            if (cameraSelection == 0) {
                robot.getOpmode().telemetry.addData("camera selected: ", "front webcam");
            }
            else {
                robot.getOpmode().telemetry.addData("camera selected: ", "arm webcam");
            }

            robot.getOpmode().telemetry.update();

        }

        // Disable Tracking when we are done;
        robot.vision.getTargetsSkyStone().deactivate();

    }
}