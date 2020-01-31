package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.SubSystems.Robot;

import java.io.IOException;


@Autonomous(name = "AutoEncoder")
@Disabled
public class AutoEncoder extends LinearOpMode {
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    // DcMotor robot.drive.frontRight, robot.drive.frontLeft, robot.drive.rearRight, robot.drive.rearLeft;
    private Robot robot;
    private ElapsedTime timer;

    // define robot position global variables
    private double robotCurrentPosX;    // unit in inches
    private double robotCurrentPosY;    // unit in inches
    private double robotCurrentAngle;   // unit in degrees

    private static final double DRIVE_SPEED = 0.4;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AR194VT/////AAABmV+h6Y9UMUSyo6qzOXtt2dAwNZ9bJ8m/6RSx/0vIwbrT4cjrgvkWuEXawFdMC7y6rbDpSZTcSs+eRZxKp0k43J6jKJktNwCLMF2iPA6yfQ6pNOIwOCoYIGC+uGdSi9+E+g9l7OH+zUWl6CXHyhUwbwTIFlduAIVaX0I2kpPuxJO4drMmZzEwsr7nHME98s/eNV30jACsP6yhUN/7w+CNEDcIvGM+J+16B978QXaGHa23ACXSkv0gXwLaztGPuPrLAfSd0kmnIaAgbDm0BUdTayFhVFaVU/VgvAjgZ7eT40BoOkAtvayDx+uPmjfTibskPk0n/eosVD7I2uxaBLHJ20w6xgOqCYlnWZ11axpyiECJ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {

        timer = new ElapsedTime();
        try {
            robot = new Robot(this, timer);
        } catch (IOException e) {
            e.printStackTrace();
        }
        robot.drive.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.drive.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.rearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.drive.rearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the robot driver to start the autonomous mode

        telemetry.addLine("Wait For Start");
        telemetry.update();
        waitForStart();

//        double ratio = 0.2;
//
//        robot.drive.frontLeft.setVelocity(robot.drive.getAngularVMaxNeverrest20() * ratio);
//        robot.drive.rearLeft.setVelocity((-robot.drive.getAngularVMaxNeverrest20() * ratio));
//        robot.drive.frontRight.setVelocity((-robot.drive.getAngularVMaxNeverrest20()) * ratio);
//        robot.drive.rearRight.setVelocity((robot.drive.getAngularVMaxNeverrest20()) * ratio);
//        sleep(10000);




    }

    public void setAllModes(DcMotor.RunMode mode) {
        robot.drive.frontLeft.setMode(mode);
        robot.drive.frontRight.setMode(mode);
        robot.drive.rearLeft.setMode(mode);
        robot.drive.rearRight.setMode(mode);
    }

    public boolean allBusy() {
        return robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy() && robot.drive.rearLeft.isBusy() && robot.drive.rearRight.isBusy();
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

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};

        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}