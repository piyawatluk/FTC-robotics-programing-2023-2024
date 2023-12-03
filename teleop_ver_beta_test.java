///@piyawat, Teera-as LSP robotics team 2023

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="teleop_ver_beta_test", group="Iterative Opmode")
public class teleop_ver_beta_test extends OpMode
{
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Lr = null; //left rear
    private DcMotor Rr = null; // right rear
    private DcMotor Et  = null; // arm extender
    private IMU imu = null;

    static final double INCREMENT = 0.01;  // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS_Drone = 0.82;  // Maximum rotational position (still can be change)
    static final double MAX_POS_Arm = 0.10;  // Maximum rotational position (still can be change)
    static final double MID_POS_Arm = 0.20;  // Middle rotational position
    static final double MIN_POS_Drone = 0.40;  // Minimum rotational position
    static final double MIN_POS_Arm = 0.30;  // Minimum rotational position

    double  position_Drone = (MAX_POS_Drone - MIN_POS_Drone) / 2; // start at halfway position
    double  position_Arm = (MIN_POS_Arm); // start at min position 

    @Override
    public void init() {
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    private void initializeHardware(){
        Rr = hardwareMap.get(DcMotor.class, "Rr");
        Lr = hardwareMap.get(DcMotor.class, "Lr");
        servo = hardwareMap.get(Servo.class, "Sv");
        Et  = hardwareMap.get(DcMotor.class, "Et");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    // paper drone function 
    public void drone(){ 
        boolean servo1 = gamepad1.right_bumper;
        boolean rampUp = servo1;
        Servo   servo;

        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position_Drone += INCREMENT ;
            if (position_Drone >= MAX_POS_Drone) {
                position_Drone = MAX_POS_Drone;
                rampUp = !rampUp;   // Switch ramp direction
            }
        } else {
            // Keep stepping down until we hit the min value.
            position_Drone -= INCREMENT ;
            if (position_Drone <= MIN_POS_Drone) {
                position_Drone = MIN_POS_Drone;
                rampUp = !rampUp;  // Switch ramp direction
            }
            // Set the servo to the new position and pause;
            servo.setPosition(position_Drone);
        }

        telemetry.addData("Servo Position", "%5.2f", position_Drone);
        telemetry.addData(">", "Press R1 to launch drone");
    }

    // arm extender function
    public void extender(){
        Et.setDirection(DcMotor.Direction.FORWARD);
        double extend_responseCurve = 1; // Exponent for the power function
        double extend_scalingFactor = 3; // Adjust this to control sensitivity
        double power = extend_scalingFactor * Math.pow(gamepad1.right_stick_y, extend_responseCurve);
        Et.setPower(power);
        telemetry.addData(">", "Press right stick y to extend");
    }
    
    @Override
    public void init_loop() {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Lr.setDirection(DcMotor.Direction.REVERSE);
        Rr.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double foward_drive = gamepad1.right_trigger;
        double backward_drive = gamepad1.left_trigger;
        double drive_responseCurve = 1; // Exponent for the power function
        double drive_scalingFactor = 3; // Adjust this to control sensitivity

        // Apply the power function with scaling to the input_x
        double input_x = drive_scalingFactor * Math.pow(gamepad1.left_stick_x, drive_responseCurve);

        // Combine drive and turn to calculate left and right powers
        leftPower = Range.clip(foward_drive - backward_drive + input_x, -1.0, 1.0);
        rightPower = Range.clip(foward_drive - backward_drive - input_x, -1.0, 1.0);

        // control hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // imu declaration 
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // reset angle
        boolean reset = gamepad1.left_bumper;

        if (reset) {
            imu.resetYaw();
        }

        drone();
        extender();

        Lr.setPower(leftPower);
        Rr.setPower(rightPower);

        // Display the current value
        telemetry.addData("Status: ", "Run Time: " + runtime.toString());
        telemetry.addLine("press L1 for yaw reset");
        telemetry.addData("left motor feed", "%.1f", leftPower);
        //telemetry.addData("right motor feed","%.2f", Rrps);
        telemetry.addData("right motor feed", "%.1f", rightPower);
        telemetry.addData("input_x", "%.1f", input_x);
        telemetry.addData("foward_drive", "%.1f", foward_drive);
        telemetry.addData("backward_drive", "%.1f", backward_drive);
        //telemetry.addData("speed", "%.2f", Lrps, Rrps);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("pitch (x)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData(">>", "Stopped");
    }
}
