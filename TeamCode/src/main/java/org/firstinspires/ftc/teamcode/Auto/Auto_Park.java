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
 * SkyStone Auto Mode for parking only
 * Created by Andrew Chiang on 1/31/2020
 */
@Autonomous(name = "Auto_Park")
public class Auto_Park extends LinearOpMode {
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

    private void initOpMode() throws IOException {
        telemetry.addData("Init Robot", "");
        telemetry.update();
        timer = new ElapsedTime();
        // visionMode 0: no camera
        this.robot = new Robot(this, timer, 0);
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

        sleep(1000);
        mainClawRotationAngle = robot.control.getMainClawRotationDegrees();
        telemetry.clearAll();

        // setup main arm and claw position
        mainClawRotationAngle = 90.0;
        robot.control.setMainClawRotationDegrees(mainClawRotationAngle);
        sleep(1000);
        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());
        sleep(1000);

        robot.control.setMainClawArmDegrees(robot.control.getMainArmTargetAngle());


        sleep(5000);

        robot.control.closeMainClawStone();
        robot.drive.moveBackward(250, 0.15);
        robot.drive.stop();
        sleep(1000);

    }
}
