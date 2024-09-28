package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="BasicMovement", group="TeleOp")
//@Disabled
public class BasicMovement extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for yougfmr robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft;
    private DcMotor RearLeft;
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectedTag = null;     // Used to hold the data for a detected AprilTag
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private Servo pmmA;
    private Servo drone;
    private DcMotor arm;
    int currentDegree = 0;
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;   //  /120;
    int armPosition = 819;
    private Servo pmmF;
    boolean APRIL = true;
    boolean Run = true;
    private IMU imu = null;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");
        pmmF = hardwareMap.get(Servo.class, "pmmfloor");
        pmmA = hardwareMap.get(Servo.class, "pmmA");
        arm = hardwareMap.get(DcMotor.class, "arm");
        drone = hardwareMap.get(Servo.class, "drone");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);


        initHardware();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //    max = Math.max(max, Math.abs(armPower));

            if (gamepad1.right_bumper) {
                if (!slowMode) {
                    slowMode = true;
                    powerMultiplier = 0.2;
                }
            } else {
                if (slowMode) {
                    slowMode = false;
                    powerMultiplier = 1.0;
                }
            }
        }
    }
    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private int armMovementBack(int degree, double power) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
            degree = currentDegree;
            telemetry.addData("Arm Position:", currentDegree);
            telemetry.update();
            break;
        }
        return currentDegree;
    }
    private int armMovement(int degree) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmPowerCalc(degree);
            degree = currentDegree;
            telemetry.addData("Arm Position:", currentDegree);
            telemetry.update();
            break;
        }
        return currentDegree;
    }

    public double getDegree() {
        double armPos;
        armPos = arm.getCurrentPosition()/TICKS_PER_DEGREE;
        return armPos;
    }


    public void ArmPowerCalc(int CalcDegree) {
        double ArmMult = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getDegree() - CalcDegree) > 0.5) { // Allow for a small error margin

            double error = CalcDegree - getDegree(); // Calculate the error

            // Calculate wheel powers based on error
            double armPower = error * ArmMult;
            double Low = 0.2;
            if(armPower<Low)
                armPower = Low;
            arm.setPower(armPower);
        }
    }
}