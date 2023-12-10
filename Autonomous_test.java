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
//all unit in this code are metric

@Autonomous(name = "autonomous test", group = "Robot")
public class Auto_test extends LinearOpMode {
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
    public void runOpMode(){
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

    public void encoderDrive(double speed,
                             double distance,
                             double timeoutS,
                             double ki,
                             double kp,
                             double kd) {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftdrive.getCurrentPosition() + (int) (distance * COUNTS_PER_METER);
            newRightTarget = rightdrive.getCurrentPosition() + (int) (distance * COUNTS_PER_METER);
            leftdrive.setTargetPosition(newLeftTarget);
            rightdrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftdrive.setPower(Math.abs(speed));
            rightdrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftdrive.isBusy() && rightdrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  "at %7d :%7d", leftdrive.getCurrentPosition(), rightdrive.getCurrentPosition());
                telemetry.addData("PID Coefficients", "Kp: %.2f, Ki: %.2f, Kd: %.2f", kp, ki, kd);
                telemetry.update();
            }

            // Stop all motion;
            leftdrive.setPower(0);
            rightdrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}

