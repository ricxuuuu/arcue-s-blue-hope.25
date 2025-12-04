package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Invokation_of_a_False_Life {
    //create motor, IMU, servo, etc. objects
    public DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    public DcMotorEx flywheel;
    public IMU imu;
    public Servo hood, flicker;

    //create static variables to represent the strings, ease of change.
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String INTAKE = "intake";
    private static final String FLYWHEEL = "flywheel";
    private static final String HOOD = "hood";
    private static final String FLICKER = "flicker";
    private static final String IMU = "imu";

    public void init(HardwareMap hwMap){
        //drivetrain
        frontLeft = hwMap.get(DcMotor.class, FRONT_LEFT);
        frontRight = hwMap.get(DcMotor.class, FRONT_RIGHT);
        backLeft   = hwMap.get(DcMotor.class, BACK_LEFT);
        backRight  = hwMap.get(DcMotor.class, BACK_RIGHT);

        //flywheel and intake
        flywheel = hwMap.get(DcMotorEx.class, FLYWHEEL);
        intake = hwMap.get(DcMotor.class, INTAKE);
    };

}
