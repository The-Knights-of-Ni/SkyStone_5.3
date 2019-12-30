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
 * Created by tarunsingh on 12/5/17.
 */

@TeleOp(name="Servo Test")
public class ServoTest extends LinearOpMode {
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
        robot.fClawL.setPosition(0);
        robot.fClawR.setPosition(0);

        while(opModeIsActive()){
            telemetry.addData("fClawR", robot.fClawR.getPosition());
            telemetry.addData("fClawL", robot.fClawL.getPosition());
            telemetry.update();
        }

    }
}