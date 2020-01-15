//package org.firstinspires.ftc.teamcode.Teleop;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.teamcode.SubSystems.Robot;
//
//@TeleOp(name = "IMUTeleTest")
//public class IMUTeleopTest extends LinearOpMode{
//    private Robot robot;
//    Orientation lastAngles = new Orientation();
//    Velocity velocity = new Velocity();
//    Position position = new Position();
//    double globalAngle, power = .30;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initOpMode();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !robot.drive.imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }
//
//
//        // wait for start button.
//
//        telemetry.addData("Mode", "Waiting for Start");
//        telemetry.update();
//        waitForStart();
//        telemetry.clearAll();
//
//        robot.drive.imu.startAccelerationIntegration(position, velocity, 1);
//        while(opModeIsActive()) {
//
//            //Get gamepad inputs
//            double leftStickX = gamepad1.left_stick_x;
//            double leftStickY = -gamepad1.left_stick_y;
//            double rightStickX = gamepad1.right_stick_x;
//            boolean aButton = gamepad1.a;
//            boolean bButton = gamepad1.b;
//            boolean dPadUp = gamepad1.dpad_up;
//            boolean dPadDown = gamepad1.dpad_down;
//            boolean dPadLeft = gamepad1.dpad_left;
//            boolean dPadRight = gamepad1.dpad_right;
//
//            double leftStickX2 = gamepad2.left_stick_x;
//            double leftStickY2 = -gamepad2.left_stick_y;
//            double rightStickX2 = gamepad2.right_stick_x;
//            double rightStickY2 = gamepad2.right_stick_y;
//            boolean aButton2 = gamepad2.a;
//            boolean bButton2 = gamepad2.b;
//            boolean dPadUp2 = gamepad2.dpad_up;
//            boolean dPadDown2 = gamepad2.dpad_down;
//            boolean dPadLeft2 = gamepad2.dpad_left;
//            boolean dPadRight2 = gamepad2.dpad_right;
//            boolean bumperLeft2 = gamepad2.left_bumper;
//            boolean bumperRight2 = gamepad2.right_bumper;
//
//
//            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
//            robot.rearLeftDriveMotor.setPower(motorPowers[0]);
//            robot.frontLeftDriveMotor.setPower(motorPowers[1]);
//            robot.rearRightDriveMotor.setPower(motorPowers[2]);
//            robot.frontRightDriveMotor.setPower(motorPowers[3]);
//
//
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 position", position.toString());
//
////            telemetry.addData("Left Stick Y2", leftStickY2);
////            telemetry.addData("Right Stick Y2", rightStickY2);
////            telemetry.addData("Right Stick X", rightStickX);
////
////            telemetry.addData("", "");
////            telemetry.addData("Left Rear Power", robot.rearLeftDriveMotor.getPower());
////            telemetry.addData("Left Front Power", robot.frontLeftDriveMotor.getPower());
////            telemetry.addData("Right Rear Power", robot.rearRightDriveMotor.getPower());
////            telemetry.addData("Right Front Power", robot.frontRightDriveMotor.getPower());
//            telemetry.update();
//
//            resetAngle();
//            updatePosition();
//            updateVelocity();
//        }
//        robot.drive.imu.stopAccelerationIntegration();
//    }
//    private void initOpMode() {
//        //Initialize DC motor objects
//        ElapsedTime timer = new ElapsedTime();
//        robot = new Robot(this, timer);
////        robot.xRailWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.xRailWinch.setTargetPosition(0);
////        robot.xRailWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.init();
////        lrDrive = hardwareMap.dcMotor.get("lrDrive");
////        lfDrive = hardwareMap.dcMotor.get("lfDrive");
////        rrDrive = hardwareMap.dcMotor.get("rrDrive");
////        rfDrive = hardwareMap.dcMotor.get("rfDrive");
//
//        //Set directions
////        lrDrive.setDirection(DcMotor.Direction.REVERSE);
////        lfDrive.setDirection(DcMotor.Direction.REVERSE);
////        rrDrive.setDirection(DcMotor.Direction.FORWARD);
////        rfDrive.setDirection(DcMotor.Direction.FORWARD);
//    }
//
//    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
//        double r = Math.hypot(leftStickX, leftStickY);
//        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
//        double lrPower = r * Math.sin(robotAngle) + rightStickX;
//        double lfPower = r * Math.cos(robotAngle) + rightStickX;
//        double rrPower = r * Math.cos(robotAngle) - rightStickX;
//        double rfPower = r * Math.sin(robotAngle) - rightStickX;
//        return new double[]{lrPower, lfPower, rrPower, rfPower};
//    }
//
//    private double calcWinchPower(double leftStickY2, double maxPower){
//        double power;
//        if(leftStickY2 > maxPower){
//            power = maxPower;
//        }
//        else if(leftStickY2 < -maxPower){
//            power = -maxPower;
//        }
//        else
//        {
//            power = Math.round(leftStickY2 * 100.0) / 100.0;
//        }
//        return power;
//    }
//
//    /**
//     * Resets the cumulative angle tracking to zero.
//     */
//    private void resetAngle()
//    {
//        lastAngles = robot.drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    public void updatePosition()
//    {
//        position = robot.drive.imu.getPosition();
//    }
//
//    public void updateVelocity()
//    {
//        velocity = robot.drive.imu.getVelocity();
//    }
//
//    /**
//     * Get current cumulative angle rotation from last reset.
//     * @return Angle in degrees. + = left, - = right.
//     */
//    private double getAngle()
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = robot.drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//
//}
