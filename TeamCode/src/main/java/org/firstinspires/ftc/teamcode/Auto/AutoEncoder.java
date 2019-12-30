package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

@Autonomous(name = "AutoEncoder", group = "Concept")

public class AutoEncoder extends LinearOpMode {
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    DcMotor driveFR, driveFL, driveBR, driveBL;

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
        // Create and set localizer parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                "AR194VT/////AAABmV+h6Y9UMUSyo6qzOXtt2dAwNZ9bJ8m/6RSx/0vIwbrT4cjrgvkWuEXawFdMC7y6rbDpSZTcSs+eRZxKp0k43J6jKJktNwCLMF2iPA6yfQ6pNOIwOCoYIGC+uGdSi9+E+g9l7OH+zUWl6CXHyhUwbwTIFlduAIVaX0I2kpPuxJO4drMmZzEwsr7nHME98s/eNV30jACsP6yhUN/7w+CNEDcIvGM+J+16B978QXaGHa23ACXSkv0gXwLaztGPuPrLAfSd0kmnIaAgbDm0BUdTayFhVFaVU/VgvAjgZ7eT40BoOkAtvayDx+uPmjfTibskPk0n/eosVD7I2uxaBLHJ20w6xgOqCYlnWZ11axpyiECJ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Create localizer
        VuforiaLocalizer localizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Set up the stone target listener
        VuforiaTrackables skystone = localizer.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = skystone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackableDefaultListener stoneListener =
                (VuforiaTrackableDefaultListener)stoneTarget.getListener();

        // Active the listeners
        // Declare motors
        skystone.activate();

        driveFL = hardwareMap.dcMotor.get("fl");
        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFR = hardwareMap.dcMotor.get("fr");
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL = hardwareMap.dcMotor.get("bl");
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBR = hardwareMap.dcMotor.get("br");
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int TICKS_PER_REV = 1120;

        // Wait for the robot driver to start the autonomous mode
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        driveFL.setTargetPosition(500);
        driveBL.setTargetPosition(500);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // find what out this is doing
//        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFL.setPower(0.2);
        driveFR.setPower(0.2);
        driveBL.setPower(0.2);
        driveBR.setPower(0.2);

        while (opModeIsActive() && driveFL.isBusy())
        {
            idle();
        }
//        while (opModeIsActive() && stoneListener.getRawPose() == null) {
//            idle();
//        }

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        // wait 5 sec so you can observe the final encoder position.
        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-end", driveFL.getCurrentPosition() + "  busy=" + driveFL.isBusy());
            telemetry.update();
            idle();
        }

        // Now back up to starting point. In this example instead of
        // having the motor monitor the encoder, we will monitor the encoder ourselves.
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveFL.setPower(-0.25);
        driveFR.setPower(-0.25);
        driveBL.setPower(-0.25);
        driveBR.setPower(-0.25);

        while (opModeIsActive() && driveFL.getCurrentPosition() > 0)
        {
            telemetry.addData("encoder-back", driveFL.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        // wait 5 sec so you can observe the final encoder position.
        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-end", driveFL.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // analyze skystone

        VectorF angles = anglesFromTarget (stoneListener);
        VectorF trans = navOffWall(stoneListener.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));

        if(trans.get(0) >0){
            driveFL.setPower(0.02);
            driveFR.setPower(-0.02);
            driveBL.setPower(0.02);
            driveBR.setPower(-0.02);
        } else {
            driveFL.setPower(-0.02);
            driveFR.setPower(0.02);
            driveBL.setPower(-0.02);
            driveBR.setPower(0.02);
        }

        do{
            if(stoneListener.getPose() != null) {
                trans = navOffWall(stoneListener.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500, 0, 0));
            }
            idle();
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        driveFL.setPower(0);
        driveFR.setPower(0);
        driveBL.setPower(0);
        driveBR.setPower(0);

        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setTargetPosition(1440);
        while(driveFL.isBusy() && opModeIsActive()) {
            //Loop body can be empty
        }
        driveFL.setPower(0);

        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setTargetPosition(1440);
        while(driveFR.isBusy() && opModeIsActive()) {
            //Loop body can be empty
        }
        driveFR.setPower(0);


        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setTargetPosition(1440);

        while(driveBL.isBusy() && opModeIsActive()) {
            //Loop body can be empty
        }
        driveBL.setPower(0);

        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setTargetPosition(1440);
        while(driveBR.isBusy() && opModeIsActive()) {
            //Loop body can be empty
        }
        driveBR.setPower(0);

        driveForwardDistance(0.2,200);
        sleep(500);
        driveForwardDistance(0.2,-200);
    }

    public void driveForwardDistance(double power, int distance) {
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setTargetPosition(distance);
        driveFR.setTargetPosition(distance);
        driveBL.setTargetPosition(distance);
        driveBR.setTargetPosition(distance);
        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(allBusy()) {
            // wait until target position is reached
        }

        stopDriving();
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setAllModes(DcMotor.RunMode mode) {
        driveFL.setMode(mode);
        driveFR.setMode(mode);
        driveBL.setMode(mode);
        driveBR.setMode(mode);
    }

    public boolean allBusy() {
        return driveFL.isBusy() && driveFR.isBusy() && driveBL.isBusy() && driveBR.isBusy();
    }

    public void stopDriving() {
        driveForward(0);
    }

    public void driveForward(double power) {
        driveFL.setPower(power);
        driveFR.setPower(power);
        driveBL.setPower(power);
        driveBR.setPower(power);
    }

    public void turnLeft(double power) {

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