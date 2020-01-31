package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "AutoMark1")
@Disabled
public class AutoMark1 extends LinearOpMode {
    private static final String TAG = "AutoMark1";

    private Robot robot;
    private ElapsedTime timer;

    private VuforiaLocalizer vuforia;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private TFObjectDetector tfod;

    private static final double     DRIVE_SPEED             = 0.4;
    private static final double     TURN_SPEED              = 0.3;


    // Robot initial position (left lander) (Crater side)
    private static final double     ROBOT_INIT_POS_X    = 15.0;
    private static final double     ROBOT_INIT_POS_Y    = 15.0;
    private static final double     ROBOT_INIT_ANGLE    = 45.0;


    private static final double     ROBOT_HALF_LENGTH    = 9.0;

    private static final int        MAX_TRIAL_COUNT = 6;

    // define robot position global variables
    private double robotCurrentPosX;    // unit in inches
    private double robotCurrentPosY;    // unit in inches
    private double robotCurrentAngle;   // unit in degrees

    // field coordinate polarity
    // no coordinate inversion needed for Blue Alliance
    // coordinate inversion needed for Red Alliance
    private boolean allianceRed = false;

    //Define Vuforia Nav
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        double startTime = 0.0;
        try {
            robot = new Robot(this, timer);
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            initRobot();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();

        log("Started Mark 1 Auto");

        // define robot position after landing
        // should be at (15.0, 15.0)
        robotCurrentPosX = ROBOT_INIT_POS_X;
        robotCurrentPosY = ROBOT_INIT_POS_Y;
        robotCurrentAngle = ROBOT_INIT_ANGLE;
//        calibrateRobotPos();


        if (tfod != null) {
            tfod.shutdown();
        }

        moveForward(10);

        moveRight(10);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "AR194VT/////AAABmV+h6Y9UMUSyo6qzOXtt2dAwNZ9bJ8m/6RSx/0vIwbrT4cjrgvkWuEXawFdMC7y6rbDpSZTcSs+eRZxKp0k43J6jKJktNwCLMF2iPA6yfQ6pNOIwOCoYIGC+uGdSi9+E+g9l7OH+zUWl6CXHyhUwbwTIFlduAIVaX0I2kpPuxJO4drMmZzEwsr7nHME98s/eNV30jACsP6yhUN/7w+CNEDcIvGM+J+16B978QXaGHa23ACXSkv0gXwLaztGPuPrLAfSd0kmnIaAgbDm0BUdTayFhVFaVU/VgvAjgZ7eT40BoOkAtvayDx+uPmjfTibskPk0n/eosVD7I2uxaBLHJ20w6xgOqCYlnWZ11axpyiECJ";;

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

//        /** Start tracking the data sets we care about. */
//        stonesAndChips.activate();

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    private void initRobot() throws IOException {
        robot.init();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Vuforia Init
        initVuforiaEngine();
        telemetry.addLine("Finished Vuforia Initialization.");
        telemetry.update();

        telemetry.addLine("Finished Initialization. Waiting for start.");
        telemetry.update();
        Log.d(TAG, "Finished Initialization. Waiting for start.");
    }

    private void initVuforiaEngine() {
        //Vuforia initialization

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVjXPzj/////AAABmQ0V3DHJw0P5lI39lVnXqNN+qX1uwVniSS5pN2zeI7ng4z9OkAMad+79Zv+vPtirvt1/Ai6dD+bZL04LynwBqdGmNSXaTXzHd21vpZdiBxmGt9Gb6nMP/p2gTc5wU6hVRJqTe+KexOqzppYs79i5rGbbwO7bZUxpXR5tJeLzicXi3prSnh49SK+kxyTX9XfsjG90+H2TfzVjpYhbX26Qi/abV4uMn7xgzC1q7L54Caixa1aytY3F/NnWAC+87mG5ghf4tcH0CPVFoYEUa0wKMG1bMWOPSfyRG/BBWdaxd1bsIU0xhI5i24nr5LXIrw2JI286TduItR/IH4WRonVA6tbz9QuuhSLlDocIgbwxIbJB";
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */
        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable redTarget = stonesAndChips.get(0);
        redTarget.setName("RedTarget");  // Stones

        VuforiaTrackable blueTarget  = stonesAndChips.get(1);
        blueTarget.setName("BlueTarget");  // Chips

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(stonesAndChips);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget.setLocation(redTargetLocationOnField);
        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));

        /*
         * To place the Stones Target on the Blue Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Finally, we translate it along the Y axis towards the blue audience wall.
         */
        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget.setLocation(blueTargetLocationOnField);
        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

    }

    private void log(String message) {
        telemetry.addLine(message);
        telemetry.update();
        Log.d(TAG, message);
    }

    private double[] robotPosNav() {
        /** Start tracking the data sets we care about. */
        targetsSkyStone.activate();

        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.update();
            return new double[]{translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch, rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle};
        }
        else {
            telemetry.addData("Visible Target", "none");
            telemetry.update();
            return null;
        }
    }

    private void calibrateRobotPos() {
        double[] robotPosCalibrationMTRX = robotPosNav();

        if (robotPosCalibrationMTRX != null) {
            // vision target found
            telemetry.addData("old position", "{X, Y, theta} = %.2f, %.2f, %.1f",
                    robotCurrentPosX, robotCurrentPosY, robotCurrentAngle);
            if (allianceRed) {
                // invert coordinate for Red Alliance
                robotCurrentPosX = - robotPosCalibrationMTRX[0];
                robotCurrentPosY = - robotPosCalibrationMTRX[1];
                robotCurrentAngle = robotPosCalibrationMTRX[5] + 180.0;
                if (robotCurrentAngle >= 360.0) robotCurrentAngle -= 360.0;
            }
            else {
                // no need to invert coordinate for Blue Alliance
                robotCurrentPosX = robotPosCalibrationMTRX[0];
                robotCurrentPosY = robotPosCalibrationMTRX[1];
                robotCurrentAngle = robotPosCalibrationMTRX[5];
            }
            telemetry.addData("new position", "{X, Y, theta} = %.2f, %.2f, %.1f",
                    robotCurrentPosX, robotCurrentPosY, robotCurrentAngle);
            telemetry.update();
        }
        else {
            // vision targets were not found
            // no change to robot position information
            telemetry.addData("Visible Target", "not found");
            telemetry.update();
        }
    }

    private void turnRobot(double degrees) {
        robot.drive.turnByAngle(TURN_SPEED, degrees);
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        telemetry.addData("turnRobot",  "turn to %7.2f degrees", robotCurrentAngle);
        telemetry.update();
        sleep(100);
    }

    private void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double  deltaX = targetPositionX - robotCurrentPosX;    // in absolute field coordinate
        double  deltaY = targetPositionY - robotCurrentPosY;    // in absolute field coordinate
        double  distanceCountX, distanceCountY;  // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX = deltaX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0)
                + deltaY * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        distanceCountY = deltaX * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + deltaY * Math.sin(robotCurrentAngle*Math.PI/180.0);
        robot.drive.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        telemetry.addData("moveToPosABS",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        robot.drive.moveToPos2D(DRIVE_SPEED, targetPositionX, targetPositionY);
        robotCurrentPosX += targetPositionY * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += targetPositionY * Math.sin(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveToPosREL",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveForward(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, 0.0, distance);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveBackward(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, 0.0, -distance);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveLeft(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, -distance, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveRight(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, distance, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

}