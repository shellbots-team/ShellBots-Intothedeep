/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="SpecimenTeleOp", group="Iterative OpMode")

public class ShellBots0mni extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Motors
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor ViperSlideL = null;
    private DcMotor ViperSlideR = null;
    private DcMotor Arm2Motor = null;
    private DcMotor SlideMotor = null;

    // Servos
    private Servo RightClaw = null;
    private Servo LeftClaw = null;
    private Servo RightWheel = null;
    private Servo LeftWheel = null;

    private double speed = 1;
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

        // Motors
        leftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        ViperSlideL = hardwareMap.get(DcMotor.class, "ViperSlideL");
        ViperSlideR = hardwareMap.get(DcMotor.class, "ViperSlideR");
        Arm2Motor = hardwareMap.get(DcMotor.class, "Arm2Motor");
        SlideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");

        // Servos
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightWheel = hardwareMap.get(Servo.class, "RightWheel");
        LeftWheel = hardwareMap.get(Servo.class, "LeftWheel");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        ViperSlideL.setDirection(DcMotor.Direction.REVERSE);
        ViperSlideR.setDirection(DcMotor.Direction.FORWARD);
        Arm2Motor.setDirection(DcMotor.Direction.FORWARD);
        SlideMotor.setDirection(DcMotor.Direction.FORWARD);

        //This should lock the motors when they are not being run YW Shellbooties
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        telemetry.addData("Motors", String.format("Front Right Power(%.2f)", ViperSlideL.getPower()));
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
        motorPowers[1] = (leftY - leftX + rightX);
        motorPowers[2] = (leftY - leftX - rightX);
        motorPowers[3] = (leftY + leftX - rightX);


        {

            {

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
                    if (abs < 0.025) {
                        motorPowers[i] = 0.0f;
                    }
                    if (abs > 1.0) {
                        motorPowers[i] /= abs;


                    }
                }
            }

            setIndividualPowers(motorPowers);
            if (this.gamepad1.y) {
                runtime.reset();
                SlideMotor.setPower(-0.7);
                while (runtime.seconds() <1.05 ) {

                    ViperSlideL.setPower(1);
                    ViperSlideR.setPower(1);
                }
                ViperSlideL.setPower(0.15);
                ViperSlideR.setPower(0.15);
                runtime.reset();
                while (runtime.seconds() <.4 ) {

                    ViperSlideL.setPower(1);
                    ViperSlideR.setPower(1);
                    Arm2Motor.setPower(-1);
                }
                ViperSlideL.setPower(0.15);
                ViperSlideR.setPower(0.15);
                runtime.reset();
                while (runtime.seconds() <.2 ) {
                    Arm2Motor.setPower(-1);
                }
                runtime.reset();
                while (runtime.seconds() <.1 ) {
                }
                runtime.reset();
                while (runtime.seconds() <.8 ) {

                    RightWheel.setPosition(0.8);
                    LeftWheel.setPosition(0.8);
                    Arm2Motor.setPower(-.2);
                }
                runtime.reset();
                RightWheel.setPosition(0.5);
                LeftWheel.setPosition(0.5);
                while (runtime.seconds() <.3 ) {

                    Arm2Motor.setPower(1);
                }
                runtime.reset();
                SlideMotor.setPower(0);
                while (runtime.seconds() <.15 ) {
                    Arm2Motor.setPower(1);
                    ViperSlideL.setPower(-.7);
                    ViperSlideR.setPower(-.7);
                }
                ViperSlideL.setPower(0.15);
                ViperSlideR.setPower(.15);
                runtime.reset();
            }
           /* if (this.gamepad1.a) {
               runtime.reset();
               while (runtime.seconds() <.9 ) {

                   ViperSlideL.setPower(1);
                   ViperSlideR.setPower(1);
               }
               ViperSlideL.setPower(0.15);
               ViperSlideR.setPower(0.15);
               runtime.reset();
               while (runtime.seconds() <1.2 ) {
                   leftFrontMotor.setPower(0.2);
                   leftBackMotor.setPower(0.2);
                   rightFrontMotor.setPower(0.2);
                   rightBackMotor.setPower(0.2);
               }
               leftFrontMotor.setPower(0);
               leftBackMotor.setPower(0);
               rightFrontMotor.setPower(0);
               rightBackMotor.setPower(0);
               runtime.reset();
               while (runtime.seconds() <.15 ) {

                   ViperSlideL.setPower(-.8);
                   ViperSlideR.setPower(-.8);
               }
               runtime.reset();

               LeftClaw.setPosition(.36);
               RightClaw.setPosition(0.22);
               while (runtime.seconds() <.2 ) {

                   ViperSlideL.setPower(-.8);
                   ViperSlideR.setPower(-.8);
                   leftFrontMotor.setPower(-0.4);
                   leftBackMotor.setPower(-0.4);
                   rightFrontMotor.setPower(-0.4);
                   rightBackMotor.setPower(-0.4);
               }
               runtime.reset();
               leftFrontMotor.setPower(0);
               leftBackMotor.setPower(0);
               rightFrontMotor.setPower(0);
               rightBackMotor.setPower(0);
               while (runtime.seconds() <1 ) {

                   ViperSlideL.setPower(-.7);
                   ViperSlideR.setPower(-.7);
               }
               ViperSlideL.setPower(0.15);
               ViperSlideR.setPower(0.15);
               runtime.reset();
           }
           if (this.gamepad1.b) {
               runtime.reset();
               while (runtime.seconds() < .2) {

                   ViperSlideL.setPower(1);
                   ViperSlideR.setPower(1);
               }
               ViperSlideL.setPower(0.15);
               ViperSlideR.setPower(0.15);
               runtime.reset();
               while (runtime.seconds() < .2) {

                   leftFrontMotor.setPower(-1);
                   leftBackMotor.setPower(-1);
                   rightFrontMotor.setPower(-1);
                   rightBackMotor.setPower(-1);
               }
               leftFrontMotor.setPower(0);
               leftBackMotor.setPower(0);
               rightFrontMotor.setPower(0);
               rightBackMotor.setPower(0);
               runtime.reset();
           }*/
            if (this.gamepad1.dpad_up) {
                //runtime.reset()
                ViperSlideL.setPower(1);
                ViperSlideR.setPower(1);
            }else if (gamepad1.dpad_down) {
                ViperSlideL.setPower(-0.5);
                ViperSlideR.setPower(-0.5);
            } else {
                ViperSlideL.setPower(0.15);
                ViperSlideR.setPower(0.15);
            }
            if (Math.abs(this.gamepad1.left_trigger)>0.1) {
                ViperSlideL.setPower(1);
                ViperSlideR.setPower(1);
            }else if (Math.abs(this.gamepad1.right_trigger)>0.1) {
                ViperSlideL.setPower(-1);
                ViperSlideR.setPower(-1);
            }else{ViperSlideL.setPower(0.15);
                ViperSlideR.setPower(0.15);
            }
            if (this.gamepad1.left_bumper) {
                LeftClaw.setPosition(.36);
                //RightClaw.setPosition(0.22);
                //runtime.reset()

            } else if (gamepad1.right_bumper) {
                //RightClaw.setPosition(0.39);
                LeftClaw.setPosition(0.18);
            }

            if (this.gamepad2.dpad_down) {
                Arm2Motor.setPower(0.65);
            } else if (this.gamepad2.dpad_up) {
                Arm2Motor.setPower(-0.5);
            } else {
                Arm2Motor.setPower(.05);
            }
            if (this.gamepad2.left_bumper) {
                RightWheel.setPosition(0.8);
                LeftWheel.setPosition(0.8);
                //runtime.reset()

            } else if (this.gamepad2.right_bumper) {
                RightWheel.setPosition(0.2);
                LeftWheel.setPosition(0.2);

            } else {
                RightWheel.setPosition(0.5);
                LeftWheel.setPosition(0.5);


                if (this.gamepad2.y) {
                    //runtime.reset()
                    SlideMotor.setPower(0.3);
                } else if (gamepad2.a) {
                    SlideMotor.setPower(-0.5);
                    {


                    }
                }}}}}
