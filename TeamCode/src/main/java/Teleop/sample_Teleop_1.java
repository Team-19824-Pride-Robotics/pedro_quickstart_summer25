package Teleop;

import android.util.Range;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "sample_Teleop_1")
@Config
public class sample_Teleop_1 extends OpMode {

    DcMotorEx arm;
    DcMotorEx tube;
    Servo claw;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    //arm, tube, and claw values
    public static int tube_speed = 20;
    public static int tube_min = 20;
    public static int tube_max = 300;

    public static int arm_pickup = 1100;
    public static int arm_score = 700;
    public static int arm_hover = 1000;

    public static double claw_open = 0;
    public static double claw_close = 0.5;

    //set the initial values for the mechanisms
    public static int arm_target = 100;
    public static int tube_target = 20;
    public static double claw_target = 0.5;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        tube = hardwareMap.get(DcMotorEx.class, "tube");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        tube.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * 0.75, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.75, true);
        follower.update();

        /*******************************************************
        Here are the mechanism controls for the arm, tube, and claw
         *******************************************************/

        //the dpad is used to move the box tube in or out
        if(gamepad2.dpad_up) {
            tube_target = tube_max;

        }
        if(gamepad2.dpad_down) {
            tube_target = tube_min;

        }


        //the buttons are used to send the arm to its positions
        if(gamepad2.x) {
            arm_target = arm_hover;
        }
        if(gamepad2.y) {
            arm_target = arm_score;
        }
        if(gamepad2.a) {
            arm_target = arm_pickup;
        }

        //the bumpers open and close the claw
        if(gamepad2.right_bumper) {
            claw_target = claw_close;
        }
        if(gamepad2.left_bumper) {
            claw_target = claw_open;
        }


        //send the arm and tube to the current target variables
        arm.setTargetPosition(arm_target);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        tube.setTargetPosition(tube_target);
        tube.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        tube.setPower(1);

        //send the claw to the current target position
        claw.setPosition(claw_target);

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}