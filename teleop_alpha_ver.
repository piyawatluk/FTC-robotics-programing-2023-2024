///@piyawat luknatin LSP robotics team 2023



package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;




@TeleOp(name="Teleop mode alpha ver.", group="Iterative Opmode")

public class Test extends OpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Lf = null;
    private DcMotor Rf = null;
    private DcMotor Lr = null;
    private DcMotor Rr = null;
    private IMU imu = null;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   1;     // period of each cycle
    static final double MAX_POS     =  0.82;     // Maximum rotational position
    static final double MIN_POS     =  0.4;     // Minimum rotational position

    // Define class members
    Servo   servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        int ver = imu.getVersion();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("software: teleop version >>> 0.1");
        telemetry.addData("IMU version >> ", ver);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Lf = hardwareMap.get(DcMotor.class, "Lf");
        Rf = hardwareMap.get(DcMotor.class, "Rf");
        Rr = hardwareMap.get(DcMotor.class, "Rr");
        Lr = hardwareMap.get(DcMotor.class, "Lr");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Lf.setDirection(DcMotor.Direction.REVERSE);
        Lr.setDirection(DcMotor.Direction.REVERSE);
        Rr.setDirection(DcMotor.Direction.FORWARD);
        Rf.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("teleop mode is arm: recheck device coniguration before continue");
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
        servo = hardwareMap.get(Servo.class, "Sv");
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive_f = gamepad1.right_trigger - 1.5;
        double drive_b = gamepad1.left_trigger - 1.5;
        double responseCurve = 1.5; // Exponent for the power function
        double scalingFactor = 0.65; // Adjust this to control sensitivity

// Apply the power function with scaling to the input_x
        double input_x = scalingFactor * Math.pow(gamepad1.left_stick_x, responseCurve);
        double turn = input_x;

// Response curve parameters


// Combine drive and turn to calculate left and right powers
        leftPower = Range.clip(drive_f - drive_b + turn, -1.0, 1.0);
        rightPower = Range.clip(drive_f - drive_b - turn, -1.0, 1.0);

        boolean servo1 = gamepad1.right_bumper;
        boolean reset = gamepad1.left_bumper;
        boolean rampUp = servo1;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity avv = imu.getRobotAngularVelocity();
        imu.resetYaw();

        if (reset) {
            imu.resetYaw();
        }

        String Lp = Double.toString(leftPower);
        String Rp = Double.toString(rightPower);

        String Lrps = Lp;
        String Rrps = Rp;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;
        // Send calculated power to wheels
        Lr.setPower(leftPower);
        Rr.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status: ", "Run Time: " + runtime.toString());
        telemetry.addData("speed", "%.2f", Lrps, Rrps);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("robot angular velocity", "%.2f M/S", avv.zRotationRate);

        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }

        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData(">", "Press R1 to shoot!!!" );
        telemetry.update();

        // Set the servo to the new position and pause;
        servo.setPosition(position);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
