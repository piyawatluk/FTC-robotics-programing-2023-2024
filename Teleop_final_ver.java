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


@TeleOp(name="Teleop_with_arm_test", group="Iterative Opmode")
public class Debug_2 extends OpMode
{
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Lr = null; //left rear
    private DcMotor Rr = null; // right rear
    private DcMotor Et  = null; // arm extender
    private IMU imu = null;

    static final double INCREMENT = 0.01;  // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS_Drone = 0.1;  // Maximum rotational position (still can be change)
    static final double MIN_POS_Drone = 0.80;  // Minimum rotational position
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_CENTI   = 6 ;     // For figuring circumference
    static final double     COUNTS_PER_METER         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CENTI * 3.1415);
    static final double     DRIVE_SPEED             = 1;

    

    public void insert(){
        Servo arm = hardwareMap.get(Servo.class, "Ar");
        Servo wrist = hardwareMap.get(Servo.class, "Wr");
        Servo hand = hardwareMap.get(Servo.class, "Ha");
        hand.setPosition(1);
        arm.setPosition(1);
        //sleep(500);
        wrist.setPosition(0.15);
        //sleep(1000);
        //arm.setPosition(1);
        //sleep(1000);
    }

    public void grip(){
        Servo arm = hardwareMap.get(Servo.class, "Ar");
        Servo wrist = hardwareMap.get(Servo.class, "Wr");
        Servo hand = hardwareMap.get(Servo.class, "Ha");
        if (gamepad2.right_bumper){
            hand.setPosition(0.2);
        }

        if (gamepad2.left_bumper){
            hand.setPosition(1);
        }
        
    }
    
    public void arm1(){
        Servo arm = hardwareMap.get(Servo.class, "Ar");
        Servo wrist = hardwareMap.get(Servo.class, "Wr");
        Servo hand = hardwareMap.get(Servo.class, "Ha");
        if (gamepad2.y){
            arm.setPosition(0);
            wrist.setPosition(0.2);
        }
    }
    
    public void arm2(){
        Servo arm = hardwareMap.get(Servo.class, "Ar");
        Servo wrist = hardwareMap.get(Servo.class, "Wr");
        Servo hand = hardwareMap.get(Servo.class, "Ha");
        if (gamepad2.x){
            arm.setPosition(1);
            wrist.setPosition(1);
        }
    }
    
    public void arm3(){
        Servo arm = hardwareMap.get(Servo.class, "Ar");
        Servo wrist = hardwareMap.get(Servo.class, "Wr");
        Servo hand = hardwareMap.get(Servo.class, "Ha");
        if (gamepad2.b){
            arm.setPosition(1);
            wrist.setPosition(0.5);
            wrist.setPosition(0.2);
        }
        
    }





    public void encoderDrive(double speed,
                             double distance,
                             double timeoutS) {
        int newEtTarget;


        // Ensure that the OpMode is still active

        // Determine new target position, and pass to motor controller
        newEtTarget = Et.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        Et.setTargetPosition(newEtTarget);


        // Turn On RUN_TO_POSITION
        Et.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        Et.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                (Et.isBusy())){


                // Display it for the driver.
            telemetry.addData("Running to",  " %7d", newEtTarget);
            telemetry.addData("Currently at",  " at %7d", Et.getCurrentPosition());
            telemetry.update();
        }

            // Stop all motion;
        Et.setPower(0);

            // Turn off RUN_TO_POSITION
        Et.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // optional pause after each move.
    }




    public void init() {
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    // arm extender function
    public void extender() {
    Et = hardwareMap.get(DcMotor.class, "Et");

    // Assuming a maximum and minimum position for your extender
    int maxPosition = 2000;  // Set your maximum position limit
    int minPosition = 0; //1000    // Set your minimum position limit

    // Get the current position of the extender
    int currentPosition = Et.getCurrentPosition();

    // Calculate the power based on the gamepad input
    double extend_responseCurve = 1; // Exponent for the power function
    double extend_scalingFactor = 3; // Adjust this to control sensitivity
    double power = extend_scalingFactor * Math.pow(-gamepad2.right_stick_y, extend_responseCurve);

    // Check if the extender is within the position limits
    if ((power > 0 && currentPosition < maxPosition) || (power < 0 && currentPosition > minPosition)) {
        // If within limits, set the power
        Et.setPower(power);
    } else {
        // If beyond limits, stop the motor
        Et.setPower(0);
    }

    telemetry.addData(">", "Press right stick y to extend");
    telemetry.addData("Current Position", currentPosition);
    telemetry.update();
}

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
        double foward_drive = gamepad1.right_trigger;
        double backward_drive = gamepad1.left_trigger;
        double drive_responseCurve = 1; // Exponent for the power function
        double drive_scalingFactor = 1; // Adjust this to control sensitivity

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
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // reset angle
        boolean reset = gamepad1.left_bumper;

        if (reset) {
            imu.resetYaw();
        }

        
        grip();
        arm1();
        arm2();
        
        if (gamepad2.a){
            insert();
        }
        


        if (gamepad1.right_bumper){
            Servo   servo;
            servo = hardwareMap.get(Servo.class, "Sv"); //declare servo position
            servo.setPosition(1);
    
        }
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
