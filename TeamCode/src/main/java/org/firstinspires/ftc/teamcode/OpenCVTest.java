package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.teamcode.Robot;

        import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Created by Andrew Chiang on 1/3/20.
 */

        import org.opencv.core.Mat;
        import org.opencv.videoio.VideoCapture;

@TeleOp(name="OpenCV Test")
public class OpenCVTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private static final float mmPerInch        = 25.4f;

    private Robot robot;

    public void initOpMode(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, timer);

    }
    public void runOpMode() {
        initOpMode();
        waitForStart();
        int maxID = 100;
        Mat frame = new Mat();

        for (int idx = 0; idx < maxID; idx++) {
            VideoCapture camera = new VideoCapture(idx);    // open the camera
            if (camera.isOpened()) {    // check if the camera is present
                camera.read(frame);
                if (frame.empty()) {
                    telemetry.addData("id %d opens: OK grabs: FAIL", idx);
                }
                else {
                    telemetry.addData("id %d opens: OK grabs: OK", idx);
                }
            }

            telemetry.update();
            camera.release();
            sleep(1000);
        }

    }
}