package org.firstinspires.ftc.teamcode.SubSystems;

import android.os.Environment;
import android.util.Log;

import com.google.ftcresearch.tfod.util.ImageUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vec3I;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by AndrewC on 12/27/2019.
 */

public class Vision {

    private ElapsedTime timer;
    private HardwareMap hardwareMap;

    private VuforiaLocalizer vuforiaWebcam = null;
    private WebcamName frontWebcamName = null;
    private WebcamName backWebcamName = null;
    private WebcamName armWebcamName = null;
    private OpenCvCamera frontWebcam;
    private OpenCvCamera armWebcam;

    public static Mat frameBuffer1 = new Mat();
    public static Mat frameBuffer2 = new Mat();
    public static Mat yCbCrChan1Mat = new Mat();
    public static Mat yCbCrChan2Mat = new Mat();
    public static Mat thresholdMat = new Mat();
    public static Mat contoursOnFrameMat = new Mat();
    public static List<MatOfPoint> contoursList = new ArrayList<>();
    public static int numContoursFound;
    private static int redColorThreshold = 160;
    private static int blueColorThreshold = 150;
    private static int yellowColorThreshold1 = 65;
    private static int yellowColorThreshold2 = 65;
    enum DetectedColor {
        RED,
        BLUE,
        YELLOW1,
        YELLOW2,
    }
    private static DetectedColor detectedColor = DetectedColor.YELLOW1;

    enum TappingMode {
        VIEW,
        THRESHOLD,
    }
    private static TappingMode tappingMode = TappingMode.VIEW;

    int[] viewportContainerIds;

    private int systemVisionMode;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    final int CAMERA_FORWARD_DISPLACEMENT  = 150;   // eg: Camera is 150 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 118;     // eg: Camera is 118 mm left of the robot's center line

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private Robot robot;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsSkyStone;

    public Vision(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        setupCameraNames();
        systemVisionMode = 2;
        setupViewports(1);
        initVuforiaEngine(0);
    }

    public Vision(HardwareMap hardwareMap, Robot robot){
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        setupCameraNames();
        systemVisionMode = 2;
        setupViewports(1);
        initVuforiaEngine(0);
    }

    /**
     *
     * @param hardwareMap
     * @param robot
     * @param visionMode
     *          o: no camera is initialized
     *          1: only armWebcam is initialized for OpenCV
     *          2: backWebcam is initialized for Vuforia
     *          3: backWebcam is initialized for Vuforia and frontWebcam is initialized for OpenCV
     *          4: armWebcam is initialized for OpenCV and frontWebcam is initialized for OpenCV
     */
    public Vision(HardwareMap hardwareMap, Robot robot, int visionMode){
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        setupCameraNames();
        systemVisionMode = visionMode;
        switch (visionMode) {
            case 0:
                break;
            case 1:
                setupViewports(1);
                initArmWebcam(0);
                break;
            case 2:
                setupViewports(1);
                initVuforiaEngine(0);
                break;
            case 3:
                setupViewports(2);
                initVuforiaEngine(0);
                initFrontWebcam(1);
                break;
            case 4:
                setupViewports(2);
                initArmWebcam(0);
                initFrontWebcam(1);
                break;
            default:
                break;
        }
    }

    public int getVisionMode() {
        return systemVisionMode;
    }

    private void setupViewports(int numViewports) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
         * This is the only thing you need to do differently when using multiple cameras.
         * Instead of obtaining the camera monitor view and directly passing that to the
         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int)}
         * on that view in order to split that view into multiple equal-sized child views,
         * and then pass those child views to the constructor.
         */
        viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, numViewports, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
    }

    private void setupCameraNames() {
        backWebcamName = hardwareMap.get(WebcamName.class, "Webcam 3");
        frontWebcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        armWebcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    private void initVuforiaEngine(int viewportID) {
        //Vuforia initialization

        /*
         * Setup Vuforia on the webcam
         */
        VuforiaLocalizer.Parameters parametersWebcam = new VuforiaLocalizer.Parameters(viewportContainerIds[viewportID]);
        parametersWebcam.vuforiaLicenseKey = "AUey4R3/////AAABmbFoecjBlEnSh5usfx1hlc07SLGE4hI5MyuUAr+09rNNBp/u1d50TPc3ydiXin5F4zAvyFKEU2pnn8ffcyfP7lydQcM+S7FZ2MXu8uIaXI3X4LpocXI22NN5KnuM/DcnjZb+1GqT41lzVUz9HX2SzgztBYDBPBvYDmCo9OcMywWkCHE9QSvWt9P1J5n2uCMZc9ZClJiKaybVac39bK4dAM/yk4TxBpRdLKbRDBGKSqlhWbGsDYmkb770A5EU4aPKLKeiQ55BOaUx9OTENNbE/vvJQnmcHkl8uz1JGpAFIvE05IFQZXLOJlgm4JtueSn33cDD3F7n0wBVVB4+ztF9IetvlYZ9Tqx00pJRSiwNJcFF";
        parametersWebcam.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK; //required for webcam
        parametersWebcam.cameraName = backWebcamName;
        parametersWebcam.fillCameraMonitorViewParent = false;
        //  Instantiate the Vuforia engine
        vuforiaWebcam = ClassFactory.getInstance().createVuforia(parametersWebcam);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.


        //Vuforia Navigation Init
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.

        targetsSkyStone = this.vuforiaWebcam.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parametersWebcam.cameraDirection);
        }

    }

    public VuforiaTrackables getTargetsSkyStone() {
        return targetsSkyStone;
    }

    public List<VuforiaTrackable> getAllTrackables() {
        return allTrackables;
    }

    public OpenGLMatrix getLastLocation() {
        return lastLocation;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public void changeLastLocation(OpenGLMatrix location){
        lastLocation = location;
    }

    public void changeIsTargetVisible(boolean value){
        targetVisible = value;
    }

    public void vuMarkScan(){
        robot.vision.changeIsTargetVisible(false);
        for (VuforiaTrackable trackable : robot.vision.getAllTrackables()) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                robot.getOpmode().telemetry.addData("Visible Target", trackable.getName());
                robot.vision.changeIsTargetVisible(true);

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    robot.vision.changeLastLocation(robotLocationTransform);
                }
                break;
            }
        }
        // Provide feedback as to where the robot is located (if we know).
        if (robot.vision.isTargetVisible()) {
            // express position (translation) of robot in inches.
            VectorF translation = robot.vision.getLastLocation().getTranslation();
            robot.getOpmode().telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(robot.vision.getLastLocation(), EXTRINSIC, XYZ, DEGREES);
            robot.getOpmode().telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            robot.getOpmode().telemetry.addData("Visible Target", "none");
        }
//        robot.getOpmode().telemetry.update();
    }

    public void initFrontWebcam() {
        initFrontWebcam(1);
    }

    public void initFrontWebcam(int viewportID) {
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName, viewportContainerIds[viewportID]);
        frontWebcam.openCameraDevice();
        frontWebcam.setPipeline(new ExtractColor());
        frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void closeFrontWebcam() {
        frontWebcam.stopStreaming();
        frontWebcam.closeCameraDevice();
    }

    public void initArmWebcam() {
        initArmWebcam(0);
    }

    public void initArmWebcam(int viewportID) {
        armWebcam = OpenCvCameraFactory.getInstance().createWebcam(armWebcamName, viewportContainerIds[viewportID]);
        armWebcam.openCameraDevice();
        armWebcam.setPipeline(new ExtractColor());
        armWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void closeArmWebcam() {
        armWebcam.stopStreaming();
        armWebcam.closeCameraDevice();
    }

    class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);
           return input;
        }
    }

    class CopyFrameToBuffer extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(frameBuffer1);
            return input;
        }
    }

    static class ExtractColor extends OpenCvPipeline {
        enum Stage {
            YCbCr,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.YCbCr;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */
            switch (tappingMode) {
                case VIEW: {
                    int currentStageNum = stageToRenderToViewport.ordinal();
                    int nextStageNum = currentStageNum + 1;
                    if (nextStageNum >= stages.length) {
                        nextStageNum = 0;
                    }
                    stageToRenderToViewport = stages[nextStageNum];
                }
                case THRESHOLD: {
                    stageToRenderToViewport = Stage.CONTOURS_OVERLAYED_ON_FRAME;
                    switch (detectedColor) {
                        case RED: {
                            redColorThreshold = redColorThreshold + 5;
                            if (redColorThreshold > 255) {
                                redColorThreshold = 0;
                            }
                        }
                        case BLUE: {
                            blueColorThreshold = blueColorThreshold + 5;
                            if (blueColorThreshold > 255) {
                                blueColorThreshold = 0;
                            }
                        }
                        case YELLOW1: {
                            yellowColorThreshold1 = yellowColorThreshold1 - 5;
                            if (yellowColorThreshold1 < 0) {
                                yellowColorThreshold1 = 255;
                            }

                        }
                        case YELLOW2: {
                            yellowColorThreshold2 = yellowColorThreshold2 - 5;
                            if (yellowColorThreshold2 < 0) {
                                yellowColorThreshold2 = 255;
                            }

                        }
                    }
                }
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            input.copyTo(frameBuffer1);
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan1Mat, 1);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            switch (detectedColor) {
                case RED: {
                    Imgproc.threshold(yCbCrChan2Mat, thresholdMat, redColorThreshold, 255, Imgproc.THRESH_BINARY);
                }
                case BLUE: {
                    Imgproc.threshold(yCbCrChan1Mat, thresholdMat, blueColorThreshold, 255, Imgproc.THRESH_BINARY);
                }
                case YELLOW1: {
                    Imgproc.threshold(yCbCrChan2Mat, thresholdMat, yellowColorThreshold1, 255, Imgproc.THRESH_BINARY_INV);
                }
                case YELLOW2: {
                    Imgproc.threshold(yCbCrChan2Mat, thresholdMat, yellowColorThreshold2, 255, Imgproc.THRESH_BINARY_INV);
                }
                default: {
                    Imgproc.threshold(yCbCrChan2Mat, thresholdMat, yellowColorThreshold1, 255, Imgproc.THRESH_BINARY_INV);
                }
            }
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 255, 0), 2, 8);

            switch (stageToRenderToViewport) {
                case YCbCr: {
                    switch (detectedColor) {
                        case RED: {
                            return yCbCrChan2Mat;
                        }
                        case BLUE: {
                            return yCbCrChan1Mat;
                        }
                        case YELLOW1: {
                            return yCbCrChan2Mat;
                        }
                        case YELLOW2: {
                            return yCbCrChan2Mat;
                        }
                        default: {
                            return yCbCrChan2Mat;
                        }
                    }
                }
                case THRESHOLD: {
                    return thresholdMat;
                }
                case CONTOURS_OVERLAYED_ON_FRAME: {
                    return contoursOnFrameMat;
                }
                case RAW_IMAGE: {
                    return input;
                }
                default: {
                    return input;
                }
            }
        }
    }

    public int getNumContoursFound() { return numContoursFound; }

    public int getRedColorThreshold() { return redColorThreshold; }

    public int getBlueColorThreshold() { return blueColorThreshold; }

    public int getYellowColorThreshold1() { return yellowColorThreshold1; }

    public int getYellowColorThreshold2() { return yellowColorThreshold2; }

    public DetectedColor getDetectedColor() { return detectedColor; }

    public void setDetectedColor(DetectedColor color) { detectedColor = color; }

    public TappingMode getTappingMode() { return tappingMode; }

    public void setTappingMode(TappingMode mode) { tappingMode = mode; }

    /**
     * Save an image to a file
     * @param tag logging tag
     * @param mat image to save
     * @param conversionToBGR openCV code to convert to bgr
     * @param fileSuffix end of file name
     * @param time start of file name
     * @return whether or not the save was successful
     *
     * from https://github.com/rsthomp/ftcvision/blob/master/ImageUtil.java
     */
    public static boolean saveImage(String tag, Mat mat, int conversionToBGR, String fileSuffix, long time) {
        Mat bgrMat = new Mat();
        Imgproc.cvtColor(mat, bgrMat, conversionToBGR);

        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        File file = new File(path, time + "_" + fileSuffix + ".png");

        if (Imgcodecs.imwrite(file.toString(), bgrMat)){
            return true;
        } else {
            Log.e(tag, "FAILED writing image to external storage");
            return false;
        }
    }

}
