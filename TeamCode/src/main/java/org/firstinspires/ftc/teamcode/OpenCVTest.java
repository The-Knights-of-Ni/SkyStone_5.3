package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.teamcode.SubSystems.Robot;

        /**
 * Created by Andrew Chiang on 1/3/20.
 */

        import org.opencv.core.Mat;
        import org.opencv.videoio.VideoCapture;

        import java.io.IOException;

@TeleOp(name="OpenCV Test")
@Disabled
public class OpenCVTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private static final float mmPerInch        = 25.4f;

    private Robot robot;

    public void initOpMode() throws IOException {
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, timer);

    }
    public void runOpMode() {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();
        int maxID = 500;
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
            else {
                telemetry.addData("id %d opens: FAIL", idx);
            }

            telemetry.update();
            camera.release();
            sleep(400);
        }

    }
}