package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Shellbots", group="Iterative OpMode")
public class ShellBotsTeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor DroneLauncher = null;
    private DcMotor ArmMotor1 = null;
    private DcMotor ArmMotor2 = null;
    private double ArmPower = 0.5;
    private double speed = 0.75;
    //Change this value as necessary. This is the max speed the motors will go.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        DroneLauncher = hardwareMap.get(DcMotor.class, "LauncherDrone");
        ArmMotor1 = hardwareMap.get(DcMotor.class, "MotorArm1");
        ArmMotor2 = hardwareMap.get(DcMotor.class, "MotorArm2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        DroneLauncher.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor1.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor2.setDirection(DcMotor.Direction.FORWARD);

        //This should lock the motors when they are not being run YW Shellbooties
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("HYPE", "ARE! YOU! READY?!?!?!?!");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        telemetry.addData("HYPE", "LEZ GOOOOOOOO!!!");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //This refers to the functions below. Don't mess with them but do look at them.
        singleJoystickDrive();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", String.format("Back Left Power(%.2f)", leftBackMotor.getPower()));
        telemetry.addData("Motors", String.format("Back Right Power(%.2f)", rightBackMotor.getPower()));
        telemetry.addData("Motors", String.format("Front Left Power(%.2f)", leftFrontMotor.getPower()));
        telemetry.addData("Motors", String.format("Front Right Power(%.2f)", rightFrontMotor.getPower()));

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Robot Stopped");
    }

    /******************************************************************************

     These three functions do all of the math and fun stuff to make the mecanum drive work.
     There's only one thing you should have to touch ever.
     *****************************************************************************/


    public void setIndividualPowers(float[] motorPowers) {
        // This function creates an array so that the function below works.

        if (motorPowers.length != 4) {
            return;
        }
        leftFrontMotor.setPower(-motorPowers[0]);
        rightFrontMotor.setPower(-motorPowers[1]);
        leftBackMotor.setPower(-motorPowers[2]);
        rightBackMotor.setPower(-motorPowers[3]);

        //If the controls seem inverted, remove these negative signs above as necessary
    }

    private void singleJoystickDrive() {
        // We don't really know how this function works, but it makes the wheels drive, so we don't question it.
        // Don't mess with this function unless you REALLY know what you're doing.

        float rightX = -this.gamepad1.right_stick_x;
        float leftY = this.gamepad1.left_stick_y;
        float leftX = -this.gamepad1.left_stick_x;

        float[] motorPowers = new float[4];

        motorPowers[0] = (leftY + leftX + rightX);
        motorPowers[1] = (leftY - leftX - rightX);
        motorPowers[2] = (leftY - leftX + rightX);
        motorPowers[3] = (leftY + leftX - rightX);

        if (this.gamepad1.dpad_up) {

            runtime.reset();
            while (gamepad1.dpad_up && (runtime.seconds() < 1)) {
                ArmMotor2.setPower(-0.6);
            }
            ArmMotor2.setPower(0);
            runtime.reset();

        }
        if (gamepad1.dpad_left) {
            runtime.reset();
            while(runtime.seconds()<0.4) {
                DroneLauncher.setPower(-1);
            }
            while(runtime.seconds()<0.2) {
                DroneLauncher.setPower(1);
            }
        }
        DroneLauncher.setPower(0);
        runtime.reset();



        if (gamepad1.right_stick_x != 0) {
            speed = 1;
        } else {
            speed = 0.75;
        }


        //New change if no work
        if (leftX == 0 && leftY == 0) {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        } else {
            leftFrontMotor.setPower(-motorPowers[0]);
            rightFrontMotor.setPower(-motorPowers[1]);
            leftBackMotor.setPower(-motorPowers[2]);
            rightBackMotor.setPower(-motorPowers[3]);
        }
        /*///end of new stuff
        float max = getLargestAbsVal(motorPowers);
        if (max < 1) {
            max = 1;
        }*/

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= (speed / 1);

            float abs = Math.abs(motorPowers[i]);
            if (abs < 0.05) {
                motorPowers[i] = 0.0f;
            }
            if (abs > 1.0) {
                motorPowers[i] /= abs;
            }
        }


        setIndividualPowers(motorPowers);

        /*if (gamepad1.right_bumper) {
            ArmPower = 0.5;
        }
        if (gamepad1.left_bumper) {
           ArmPower = 1;
        }*/
            /*if (gamepad1.a) {
                runtime.reset();
                while (runtime.seconds() < 0.4) {
                    leftFrontMotor.setPower(1);
                    rightFrontMotor.setPower(1);
                    leftBackMotor.setPower(1);
                    rightBackMotor.setPower(1);
                }
                runtime.reset();
                while (runtime.seconds() < 0.4) {
                    leftFrontMotor.setPower(-1);
                    rightFrontMotor.setPower(-1);
                    leftBackMotor.setPower(-1);
                    rightBackMotor.setPower(-1);
                }*/
        // runtime.reset();
        //}
            /*if (gamepad1.left_bumper) {
                ArmMotor2.setPower(0);
            }
            if (gamepad1.y) {
                runtime.reset();
                while (gamepad1.y && (runtime.seconds() < 0.1)) {
                    ArmMotor2.setPower(0.6);
                }
                ArmMotor2.setPower(0);
            }
            if (gamepad1.a) {
                runtime.reset();
                while (gamepad1.a && (runtime.seconds() < 0.1)) {
                    ArmMotor2.setPower(-0.6);
                }
                ArmMotor2.setPower(0);
            }
            if (gamepad1.x) {
                runtime.reset();
                while (gamepad1.x && (runtime.seconds() < 1)) {
                    ArmMotor1.setPower(-1);
                }
                ArmMotor1.setPower(0);
            }
            if (gamepad1.b) {
                runtime.reset();
                while (gamepad1.b && (runtime.seconds() < 1)) {
                    ArmMotor1.setPower(1);
                }
                ArmMotor1.setPower(0);
            }*/
            /*if (gamepad1.dpad_down) {
                runtime.reset();
                while (runtime.seconds() < 4) {
                    ArmMotor1.setPower(1);
                }
                ArmMotor1.setPower(0);
                runtime.reset();
                while (runtime.seconds() < 1.5
                ) {
                    ArmMotor2.setPower(0.6);
                }
                runtime.reset();
                while (runtime.seconds() < 1) {
                    leftFrontMotor.setPower(-0.3);
                    rightFrontMotor.setPower(-0.3);
                    leftBackMotor.setPower(-0.3);
                    rightBackMotor.setPower(-0.3);
                }
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightBackMotor.setPower(0);
                runtime.reset();
                ArmMotor2.setPower(1);
                while (runtime.seconds() < 2.8) {
                    ArmMotor1.setPower(-1);
                }
                ArmMotor1.setPower(0);
            }*/
        if (gamepad1.a) {
            runtime.reset();
            while (gamepad1.a && (runtime.seconds() < 0.1)) {
                ArmMotor2.setPower(-0.6);
            }
            ArmMotor2.setPower(0);
        }
        if (gamepad1.y) {
            runtime.reset();
            while (gamepad1.y && (runtime.seconds() < 0.1)) {
                ArmMotor2.setPower(0.6);
            }
            ArmMotor2.setPower(0);
        }

        if (gamepad1.dpad_down){
            runtime.reset();
            while (runtime.seconds() < 4) {
                ArmMotor1.setPower(1);
            }
            ArmMotor1.setPower(0);
            runtime.reset();
            while (runtime.seconds() < 1.5) {
                ArmMotor2.setPower(0.6);
            }
            runtime.reset();
            while (runtime.seconds() < 1) {
                leftFrontMotor.setPower(-0.3);
                leftBackMotor.setPower(-0.3);
                rightBackMotor.setPower(-0.3);
                rightFrontMotor.setPower(-0.3);
            }
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            runtime.reset();
            ArmMotor2.setPower(1);
            while (runtime.seconds() < 2.8) {
                ArmMotor1.setPower(-1);
            }
            ArmMotor1.setPower(0);
        }
        if (gamepad1.a){
            ArmMotor2.setPower(0);
        }
        if (gamepad1.x){
            runtime.reset();
            while (gamepad1.x && runtime.seconds() < 1) {
                ArmMotor1.setPower(-1);
            }
            ArmMotor1.setPower(0);
        }
        if (gamepad1.b){
            runtime.reset();
            while (gamepad1.b && runtime.seconds() < 1) {
                ArmMotor1.setPower(1);
            }
            ArmMotor1.setPower(0);
        }
        if(gamepad1.right_bumper){
            runtime.reset();
            while (gamepad1.right_bumper && runtime.seconds() < 0.35){
                leftFrontMotor.setPower(0.8);
                leftBackMotor.setPower(0.8);
                rightBackMotor.setPower(0.8);
                rightFrontMotor.setPower(0.8);
            }
            runtime.reset();
            while (runtime.seconds() < 0.4){
                leftFrontMotor.setPower(-1);
                leftBackMotor.setPower(-1);
                rightBackMotor.setPower(-1);
                rightFrontMotor.setPower(-1);
            }
        }

        telemetry.addData("Arm Position", ArmMotor1.getCurrentPosition());
        telemetry.update();





   /* private float getLargestAbsVal(float[] values) {
        // This function does some math!
        float max = 0;
        for (float val : values) {
            if (Math.abs(val) > max) {
                max = Math.abs(val);*/
    }
}

