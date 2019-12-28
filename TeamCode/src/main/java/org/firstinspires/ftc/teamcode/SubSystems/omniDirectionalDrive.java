package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//>>>>>>> d827f8fea88cc58a3af4d5b4c9f323b732e67b7d


public class omniDirectionalDrive {


    //Sensors
    private BNO055IMU imu;
    private HardwareMap hardwareMap;


    public omniDirectionalDrive(OpMode opMode, ElapsedTime timer){
        hardwareMap = opMode.hardwareMap;
//        this.timer = timer;
        init();
    }
    public omniDirectionalDrive (){
        init();
    }
    public void init(){


        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        colorSensor = hardwareMap.colorSensor.get("color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu.initialize(parameters);
//        position = imu.getPosition();

        //Subsystems
//        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, imu, timer);

//>>>>>>> d827f8fea88cc58a3af4d5b4c9f323b732e67b7d
    }
}
