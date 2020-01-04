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

    boolean aButton;
    boolean isaButtonPressedPrev =false;
    boolean bButton;
    boolean isbButtonPressedPrev = false;
    boolean xButton;
    boolean isxButtonPressedPrev =false;
    boolean yButton;
    boolean isyButtonPressedPrev = false;


    public void initOpMode(){
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, timer);

    }
    public void runOpMode() {
        initOpMode();
        aButton = gamepad1.a;
        bButton = gamepad1.b;
        telemetry.addLine("Wait For Start");
        telemetry.update();
        waitForStart();
        robot.mainArm.setPosition(0.5);
        telemetry.addData("mainArm", robot.mainArm.getPosition());
        telemetry.update();
//        robot.fClawL.setPosition(0.5);
//        robot.fClawR.setPosition(0.5);
//
//        robot.mainArm.setPosition(1.0);
//        robot.mainClaw.setPosition(0.5);

        while(opModeIsActive()){
            aButton = gamepad1.a;
            bButton = gamepad1.b;
            xButton = gamepad1.x;
            yButton = gamepad1.y;
            if (aButton && !isaButtonPressedPrev && ((robot.mainArm.getPosition()+0.05) < 1.0)) {
                robot.mainArm.setPosition(robot.mainArm.getPosition() + 0.05);
                isaButtonPressedPrev = true;
            }
            if(bButton && !isbButtonPressedPrev && ((robot.mainArm.getPosition()-0.05) > 0.0)){
                robot.mainArm.setPosition(robot.mainArm.getPosition() - 0.05);
                isbButtonPressedPrev = true;
            }
            if (!aButton) {
                isaButtonPressedPrev = false;
            }
            if(!bButton){
                isbButtonPressedPrev = false;
            }

            if (xButton && !isxButtonPressedPrev) {
                robot.mainArm.setPosition(0.83);
                isxButtonPressedPrev = true;
            }
            if(yButton && !isyButtonPressedPrev){
                robot.mainArm.setPosition(0.05);
                isyButtonPressedPrev = true;
            }
            if (!xButton) {
                isxButtonPressedPrev = false;
            }
            if(!yButton){
                isyButtonPressedPrev = false;
            }


            telemetry.addData("mainArm", robot.mainArm.getPosition());
            telemetry.update();
            sleep(10);
        }

    }
}