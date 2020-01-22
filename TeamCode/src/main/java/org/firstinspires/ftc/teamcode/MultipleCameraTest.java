package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
 * method in order to concurrently display the preview of two cameras, using
 * OpenCV on both.
 */
@TeleOp (name = "MCT1")
@Disabled
public class MultipleCameraTest extends LinearOpMode
{
    OpenCvCamera phoneCam;
    OpenCvCamera webCam1;
    OpenCvCamera webCam2;
    OpenCvCamera webCam3;

    CameraCharacteristics webCam1Characteristics;
    CameraCharacteristics webCam2Characteristics;


    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at {@link InternalCameraExample} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
         * This is the only thing you need to do differently when using multiple cameras.
         * Instead of obtaining the camera monitor view and directly passing that to the
         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
         * on that view in order to split that view into multiple equal-sized child views,
         * and then pass those child views to the constructor.
         */
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        3, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        webCam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        webCam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        webCam3 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 3"), viewportContainerIds[2]);


//        phoneCam.openCameraDevice();
        webCam1.openCameraDevice();
        webCam2.openCameraDevice();
        webCam3.openCameraDevice();

//        phoneCam.setPipeline(new UselessGreenBoxDrawingPipeline());
        webCam1.setPipeline(new UselessGreenBoxDrawingPipeline());
        webCam2.setPipeline(new UselessGreenBoxDrawingPipeline());
        webCam3.setPipeline(new UselessGreenBoxDrawingPipeline());

        webCam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webCam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webCam3.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {
//            telemetry.addData("PhoneCam FPS", phoneCam.getFps());
            telemetry.addData("WebCam 1 FPS", webCam1.getFps());
            telemetry.addData("Webcam 2 FPS", webCam2.getFps());
            telemetry.addData("Webcam 3 FPS", webCam3.getFps());
            telemetry.update();
//            phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            sleep(500);
//            phoneCam.stopStreaming();
//            webCam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            sleep(500);
//            webCam1.stopStreaming();
//            webCam2.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            sleep(500);
//            webCam2.stopStreaming();

            sleep(100);
        }
//        phoneCam.closeCameraDevice();
        webCam1.closeCameraDevice();
//        webCam2.closeCameraDevice();
    }

    class UselessGreenBoxDrawingPipeline extends OpenCvPipeline
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
//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Camera;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
////import org.openftc.easyopencv.OpenCvWebcam;
//
///**
// * In this sample, we demonstrate how to use the {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
// * method in order to concurrently display the preview of two cameras, using
// * OpenCV on both.
// */
//@TeleOp (name = "MCT")
//public class MultipleCameraTest extends LinearOpMode
//{
//    OpenCvCamera phoneCam;
//    OpenCvCamera webcam;
//
//    @Override
//    public void runOpMode()
//    {
//        /**
//         * NOTE: Many comments have been omitted from this sample for the
//         * sake of conciseness. If you're just starting out with EasyOpenCV,
//         * you should take a look at {@link InternalCameraExample} or its
//         * webcam counterpart, {@link WebcamExample} first.
//         */
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        /**
//         * This is the only thing you need to do differently when using multiple cameras.
//         * Instead of obtaining the camera monitor view and directly passing that to the
//         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
//         * on that view in order to split that view into multiple equal-sized child views,
//         * and then pass those child views to the constructor.
//         */
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
//                .splitLayoutForMultipleViewports(
//                        cameraMonitorViewId, //The container we're splitting
//                        2, //The number of sub-containers to create
//                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
//
////        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
//        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
//
//
//        phoneCam.openCameraDevice();
//        webcam.openCameraDevice();
//
//        phoneCam.setPipeline(new UselessGreenBoxDrawingPipeline());
//        webcam.setPipeline(new UselessGreenBoxDrawingPipeline());
//
//        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Internal cam FPS", phoneCam.getFps());
//            telemetry.addData("Webcam FPS", webcam.getFps());
//            telemetry.update();
//
//            sleep(100);
//        }
//    }
//
//    class UselessGreenBoxDrawingPipeline extends OpenCvPipeline
//    {
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            Imgproc.rectangle(
//                    input,
//                    new Point(
//                            input.cols()/4,
//                            input.rows()/4),
//                    new Point(
//                            input.cols()*(3f/4f),
//                            input.rows()*(3f/4f)),
//                    new Scalar(0, 255, 0), 4);
//
//            return input;
//        }
//    }
//}
