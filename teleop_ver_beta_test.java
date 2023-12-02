///@piyawat luknatin LSP robotics team 2023

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



@TeleOp(name="Nig", group="Iterative Opmode")

public class Nig extends OpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Lr = null;
    private DcMotor Rr = null;
    private DcMotor Et = null;
    private IMU imu = null;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS_s     =  0.82;     // Maximum rotational position (still can be change)
    static final double MAX_POS_a     =  0.1; // Maximum rotational position (still can be change)
    static final double MID_POS_a     =  0.2;  // Middle rotational position
    static final double MIN_POS_s     =  0.4;   // Minimum rotational position
    static final double MIN_POS_a     =  0.3;   // Minimum rotational position

    double  position_s = (MAX_POS_s - MIN_POS_s) / 2; // Start at halfway position
    double  position_a = (MIN_POS_a); // start at min. pos.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    //paper drone func.
    public void shot(){
        servo = hardwareMap.get(Servo.class, "Sv"); //declare servo pos.
        boolean servo1 = gamepad1.right_bumper;
        boolean rampUp = servo1;
        Servo   servo;

        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position_s += INCREMENT ;
            if (position_s >= MAX_POS_s ) {
                position_s = MAX_POS_s;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else {

            // Keep stepping down until we hit the min value.
            position_s -= INCREMENT ;
            if (position_s <= MIN_POS_s ) {
                position_s = MIN_POS_s;
                rampUp = !rampUp;  // Switch ramp direction
            }
            // Set the servo to the new position and pause;
            servo.setPosition(position_s);
        }

        telemetry.addData("Servo Position", "%5.2f", position_s);
        telemetry.addData(">", "Press R1 to shoot!!!" );
    }

    //func. extender
    public void extender(){
        Et = hardwareMap.get(DcMotor.class, "Et");
        Et.setDirection(DcMotor.Direction.FORWARD);
        boolean extender = gamepad1.right_stick_y;
        double responseCurve_e = 1; // Exponent for the power function
        double scalingFactor_e = 3; // Adjust this to control sensitivity
        double power = scalingFactor_e * Math.pow(gamepad1.left_stick_x, responseCurve_e);
        Et.setPower(power);
        telemetry.addData(">", "Press L1 for ereaction!!!!!!");

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        Rr = hardwareMap.get(DcMotor.class, "Rr");
        Lr = hardwareMap.get(DcMotor.class, "Lr");
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
        double drive_f = gamepad1.right_trigger;
        double drive_b = gamepad1.left_trigger;
        double responseCurve_d = 1; // Exponent for the power function
        double scalingFactor_d = 3; // Adjust this to control sensitivity

// Apply the power function with scaling to the input_x
        double input_x = scalingFactor_d * Math.pow(gamepad1.left_stick_x, responseCurve_d);
        double turn = input_x;

// Combine drive and turn to calculate left and right powers
        leftPower = Range.clip(drive_f - drive_b + turn, -1.0, 1.0);
        rightPower = Range.clip(drive_f - drive_b - turn, -1.0, 1.0);

        //robot orient.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //imu declare.

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //reset angle

        boolean reset = gamepad1.left_bumper;

        if (reset) {
            imu.resetYaw();
        }

        shot();
        extender();

        Lr.setPower(leftPower);
        Rr.setPower(rightPower);

        // Display the current value
        telemetry.addData("Status: ", "Run Time: " + runtime.toString());
        telemetry.addLine("press L1 for yaw reset");
        telemetry.addData("left motor feed", "%.1f",leftPower);
        //telemetry.addData("right motor feed","%.2f", Rrps);
        telemetry.addData("right motor feed", "%.1f",rightPower);
        telemetry.addData("input_x", "%.1f", input_x);
        telemetry.addData("drive_f", "%.1f", drive_f);
        telemetry.addData("drive_b", "%.1f", drive_b);
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
        telemetry.addData(">>", "the robot has been stop");
    }

}
