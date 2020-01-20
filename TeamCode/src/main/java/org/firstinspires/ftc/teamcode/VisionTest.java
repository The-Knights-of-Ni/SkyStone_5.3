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

    boolean aButton2;
    boolean isaButton2PressedPrev = false;

    double timeCurrent;

    public void initOpMode(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, timer, 3);

    }
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        initOpMode();
        waitForStart();
        robot.vision.getTargetsSkyStone().activate();
        isaButton2PressedPrev = false;
        while (!isStopRequested()) {

//            // check all the trackable targets to see which one (if any) is visible.
//            robot.vision.changeIsTargetVisible(false);
//            for (VuforiaTrackable trackable : robot.vision.getAllTrackables()) {
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    robot.vision.changeIsTargetVisible(true);
//
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        robot.vision.changeLastLocation(robotLocationTransform);
//                    }
//                    break;
//                }
//            }
//            // Provide feedback as to where the robot is located (if we know).
//            if (robot.vision.isTargetVisible()) {
//                // express position (translation) of robot in inches.
//                VectorF translation = robot.vision.getLastLocation().getTranslation();
//                telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0), translation.get(1), translation.get(2));
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(robot.vision.getLastLocation(), EXTRINSIC, XYZ, DEGREES);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//            }
//            else {
//                telemetry.addData("Visible Target", "none");
//            }
//            telemetry.update();

            robot.vision.vuMarkScan();
            robot.getOpmode().telemetry.addData("Threshold", "{r, b, y1, y2} = %d, %d, %d, %d",
                    robot.vision.getRedColorThreshold(),robot.vision.getBlueColorThreshold(),
                    robot.vision.getYellowColorThreshold1(), robot.vision.getYellowColorThreshold2());

            timeCurrent = timer.nanoseconds();
            aButton2 = gamepad2.a;

            // save images
            if (aButton2 && !isaButton2PressedPrev) {
                robot.getOpmode().telemetry.addData("saving ", "images...");
                robot.vision.saveImage("VisionTest", robot.vision.frameBuffer1, Imgproc.COLOR_RGBA2BGR, "original", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan2Mat, Imgproc.COLOR_RGBA2BGR, "CbImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.yCbCrChan1Mat, Imgproc.COLOR_RGBA2BGR, "CrImage", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.thresholdMat, Imgproc.COLOR_RGBA2BGR, "threshold", (long) timeCurrent);
                robot.vision.saveImage("VisionTest", robot.vision.contoursOnFrameMat, Imgproc.COLOR_RGBA2BGR, "contours", (long) timeCurrent);
                isaButton2PressedPrev = true;
            }
            if (!aButton2) {
                isaButton2PressedPrev = false;
            }
            robot.getOpmode().telemetry.update();

        }

        // Disable Tracking when we are done;
        robot.vision.getTargetsSkyStone().deactivate();

    }
}