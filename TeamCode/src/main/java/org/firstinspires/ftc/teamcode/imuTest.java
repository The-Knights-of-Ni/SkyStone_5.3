//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
///**
// * Created by Elijah R. on 11/16/2019.
// */
//@Autonomous(name="IMU Test", group ="Concept")
//
//public class imuTest extends LinearOpMode {
//    public String name;
//
//    //Sensors
//    BNO055IMU imu;
//
//    //Subsystems
//
//    @Override public void runOpMode() throws InterruptedException {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        if (opModeIsActive()) {
//            telemetry.addData(String.format("Test: "), "%.03f", imu.getPosition().x);
//            while(opModeIsActive()) {
//                if (imu != null) {
//                    telemetry.addData(String.format("Position(x): "), "%.03f", imu.getPosition().x);
//                    telemetry.addData(String.format("Position(y): "), "%.00005f", imu.getPosition().y);
//                    telemetry.addData(String.format("Position(z): "), "%.03f", imu.getPosition().z);
//                }
//                telemetry.update();
//            }
//        }
//    }
//}