
        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import org.openftc.easyopencv.OpenCvInternalCamera;
        import org.openftc.easyopencv.OpenCvPipeline;

        import java.util.ArrayList;
        import java.util.List;

        import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

        /**
 * In this sample, we demonstrate how to use the {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
 * method in order to concurrently display the preview of two cameras, using
 * OpenCV on an internal camera, and Vuforia on a webcam
 */
@TeleOp (name = "MCT_Vuforia")
@Disabled
public class MultipleCameraTest_Vuforia extends LinearOpMode
{
    OpenCvCamera webcam1;
    OpenCvCamera webcam2;
    VuforiaLocalizer vuforia = null;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at {@link InternalCameraExample} or its
         * cam2 counterpart, {@link WebcamExample} first.
         *
         * Also check out {@link MultipleCameraExample}
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
         * This is the only thing you need to do differently when using multiple cameras.
         * Instead of obtaining the camera monitor view and directly passing that to the
         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int)}
         * on that view in order to split that view into multiple equal-sized child views,
         * and then pass those child views to the constructor.
         */
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Setup OpenCV on the phone camera
         */
//telemetry.addLine("Test1");
//telemetry.update();
//sleep(1000);

//        webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[1]);
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);
//telemetry.addLine("Test2");
//telemetry.update();
//sleep(1000);
        webcam1.openCameraDevice();
//telemetry.addLine("Test3");
//telemetry.update();
//sleep(1000);
        webcam1.setPipeline(new SamplePipeline());
//telemetry.addLine("Test4");
//telemetry.update();
//sleep(1000);
        webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//telemetry.addLine("Test5");
//telemetry.update();
//sleep(1000);

        /*
         * Setup Vuforia on the webcam
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "AUey4R3/////AAABmbFoecjBlEnSh5usfx1hlc07SLGE4hI5MyuUAr+09rNNBp/u1d50TPc3ydiXin5F4zAvyFKEU2pnn8ffcyfP7lydQcM+S7FZ2MXu8uIaXI3X4LpocXI22NN5KnuM/DcnjZb+1GqT41lzVUz9HX2SzgztBYDBPBvYDmCo9OcMywWkCHE9QSvWt9P1J5n2uCMZc9ZClJiKaybVac39bK4dAM/yk4TxBpRdLKbRDBGKSqlhWbGsDYmkb770A5EU4aPKLKeiQ55BOaUx9OTENNbE/vvJQnmcHkl8uz1JGpAFIvE05IFQZXLOJlgm4JtueSn33cDD3F7n0wBVVB4+ztF9IetvlYZ9Tqx00pJRSiwNJcFF";
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK; //required for webcam
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 3");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

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

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        targetsSkyStone.activate();

        waitForStart();

        webcam1.stopStreaming();
        webcam1.closeCameraDevice();
        sleep(100);

        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        webcam2.openCameraDevice();
        webcam2.setPipeline(new SamplePipeline());
        webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        while (opModeIsActive())
        {
            for (VuforiaTrackable trackable : allTrackables)
            {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
                {
                    telemetry.addData("Visible Target", trackable.getName());
                }
            }

            telemetry.addData("Internal cam 1 FPS", webcam1.getFps());
            telemetry.addData("Internal cam 2 FPS", webcam2.getFps());
            telemetry.update();

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {
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
}

