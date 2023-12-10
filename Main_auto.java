package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //call op mode
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;//call motor

import com.qualcomm.robotcore.util.ElapsedTime;//call time

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;//call imu
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Main_auto test", group = "Robot")
public class Main_auto extends LinearOpMode {
    private DcMotor leftdrive = null;
    private DcMotor rightdrive = null;
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu = null;

    //declare motor constant
    static final double  COUNTS_PER_MOTOR_REV  = 1440 ;    // eg: TETRIX Motor Encoder
    static final double  DRIVE_GEAR_REDUCTION  = 1.0 ;     // No External Gearing.
    static final double  WHEEL_DIAMETER_METER  = 0.1 ;     // For figuring circumference
    static final double  COUNTS_PER_METER  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_METER * 3.1415);
    static final double  DRIVE_SPEED  = 1;
    static final double  TURN_SPEED  = 0.5;

    @Override
    public void runOpMode() {
        leftdrive = hardwareMap.get(DcMotor.class, "Lr"); //hardware posistion declare
        rightdrive = hardwareMap.get(DcMotor.class, "Rr");
        imu = hardwareMap.get(IMU.class, "imu");
        double ki = 0.5;
        double kp = 0.5;
        double kd = 0.5;

        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftdrive.setDirection(DcMotor.Direction.REVERSE);//motor direction declare
        rightdrive.setDirection(DcMotor.Direction.FORWARD);

        leftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();//reset angle

        telemetry.addData("starting at", "%2d :%2d", leftdrive.getCurrentPosition(), rightdrive.getCurrentPosition());
        telemetry.addData("yaw (z)", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();

        // Example: Turn 180 degrees to the right
        // encoderDrive(TURN_SPEED, 180, 5.0, ki, kp, kd);
        
        encoderDrive(DRIVE_SPEED,  2, 5.0, ki, kp, kd);  // S1: Forward 47 Inches with 5 Sec timeout
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.

    }
}