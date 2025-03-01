package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2019 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "TestSpecimenAuto")
public class TestSampleAuto extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor ViperSlideL = null;
    private DcMotor ViperSlideR = null;
    private DcMotor SlideMotor = null;
    private Servo RightClaw = null;
    private Servo LeftClaw = null;
    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.35;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.35;

    @Override
    public void runOpMode() {
// Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        ViperSlideL = hardwareMap.get(DcMotor.class, "ViperSlideL");
        ViperSlideR = hardwareMap.get(DcMotor.class, "ViperSlideR");
        SlideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        ViperSlideL.setDirection(DcMotor.Direction.REVERSE);
        ViperSlideR.setDirection(DcMotor.Direction.FORWARD);
        SlideMotor.setDirection(DcMotor.Direction.FORWARD);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Shellbooty", "Big");    //
        telemetry.update();
        runtime.reset();
        waitForStart();
        runtime.reset();
            RightClaw.setPosition(0.4);
            LeftClaw.setPosition(0.1);
        SlideMotor.setPower(.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1 )) {
            leftFrontDrive.setPower(.65);
            rightFrontDrive.setPower(.65);
            rightBackDrive.setPower(-.65);
            leftBackDrive.setPower(-.65);
            ViperSlideL.setPower(1);
            ViperSlideR.setPower(1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
            while (opModeIsActive() && (runtime.seconds() <.5 )) {
                leftFrontDrive.setPower(0.2);
                rightFrontDrive.setPower(0.2);
                rightBackDrive.setPower(-0.2);
                leftBackDrive.setPower(-0.2);
            }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() <.28 )) {

                ViperSlideL.setPower(-.6);
                ViperSlideR.setPower(-.6);
            }
            runtime.reset();
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
            LeftClaw.setPosition(.36);
            RightClaw.setPosition(0.22);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
        }
        runtime.reset();
            while (opModeIsActive() && (runtime.seconds() <.2 )) {

                ViperSlideL.setPower(.15);
                ViperSlideR.setPower(.15);
                leftFrontDrive.setPower(-0.4);
                rightFrontDrive.setPower(-0.4);
                rightBackDrive.setPower(0.4);
                leftBackDrive.setPower(0.4);
            }
            runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
            leftFrontDrive.setPower(-.3);
            rightFrontDrive.setPower(-.3);
            rightBackDrive.setPower(.3);
            leftBackDrive.setPower(.3);
            ViperSlideL.setPower(-1);
            ViperSlideR.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        ViperSlideL.setPower(-.5);
        ViperSlideR.setPower(-.5);
        while (opModeIsActive() && (runtime.seconds() <.435 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        ViperSlideL.setPower(-.2);
        ViperSlideR.setPower(-.2);
        while (opModeIsActive() && (runtime.seconds() <.768 )) {
            leftFrontDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(-.7);
            leftBackDrive.setPower(-.7);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
        while (opModeIsActive() && (runtime.seconds() <.425 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.55 )) {
            leftFrontDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(-.7);
            leftBackDrive.setPower(-.7);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.35 )) {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.05 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.3 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(1);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.05 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <1 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(1);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.02 )) {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(0);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.25 )) {
            leftFrontDrive.setPower(.8);
            rightFrontDrive.setPower(.8);
            rightBackDrive.setPower(-.8);
            leftBackDrive.setPower(-.8);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.3 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-.2);
            rightBackDrive.setPower(0.2);
            leftBackDrive.setPower(-1);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.6 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(-1);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.9 )) {
            leftFrontDrive.setPower(.5);
            rightFrontDrive.setPower(.5);
            rightBackDrive.setPower(-.5);
            leftBackDrive.setPower(-.5);
        }
        runtime.reset();










        RightClaw.setPosition(0.4);
        LeftClaw.setPosition(0.1);
        runtime.reset();
        while (runtime.seconds() < .2) {

            ViperSlideL.setPower(1);
            ViperSlideR.setPower(1);
        }
        ViperSlideL.setPower(0.15);
        ViperSlideR.setPower(0.15);
        runtime.reset();
        while (runtime.seconds() < .3) {
            leftFrontDrive.setPower(-.5);
            rightFrontDrive.setPower(-.5);
            rightBackDrive.setPower(.5);
            leftBackDrive.setPower(.5);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.50 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.15 )) {
            leftFrontDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(-.7);
            leftBackDrive.setPower(-.7);
            ViperSlideL.setPower(.85);
            ViperSlideR.setPower(.85);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
        while (opModeIsActive() && (runtime.seconds() <.32 )) {
            leftFrontDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(-.7);
            leftBackDrive.setPower(-.7);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.24 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.6 )) {
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.28 )) {

            ViperSlideL.setPower(-.6);
            ViperSlideR.setPower(-.6);
        }
        runtime.reset();
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
        LeftClaw.setPosition(.36);
        RightClaw.setPosition(0.22);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.3 )) {

            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
            leftFrontDrive.setPower(-0.4);
            rightFrontDrive.setPower(-0.4);
            rightBackDrive.setPower(0.4);
            leftBackDrive.setPower(0.4);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.2 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(1);
            ViperSlideL.setPower(-1);
            ViperSlideR.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.61 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.91 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.37 )) {
            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.65 )) {
            leftFrontDrive.setPower(.9);
            rightFrontDrive.setPower(.9);
            rightBackDrive.setPower(-.9);
            leftBackDrive.setPower(-.9);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();










        RightClaw.setPosition(0.4);
        LeftClaw.setPosition(0.1);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
        }
        runtime.reset();
        while (runtime.seconds() < .2) {

            ViperSlideL.setPower(1);
            ViperSlideR.setPower(1);
        }
        ViperSlideL.setPower(0.15);
        ViperSlideR.setPower(0.15);
        runtime.reset();
        while (runtime.seconds() < .3) {
            leftFrontDrive.setPower(-.5);
            rightFrontDrive.setPower(-.5);
            rightBackDrive.setPower(.5);
            leftBackDrive.setPower(.5);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.53 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.2 )) {
            leftFrontDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(-.7);
            leftBackDrive.setPower(-.7);
            ViperSlideL.setPower(.84);
            ViperSlideR.setPower(.84);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.27 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.6 )) {
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.28 )) {

            ViperSlideL.setPower(-.6);
            ViperSlideR.setPower(-.6);
        }
        runtime.reset();
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
        LeftClaw.setPosition(.36);
        RightClaw.setPosition(0.22);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.2 )) {

            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
            leftFrontDrive.setPower(-0.4);
            rightFrontDrive.setPower(-0.4);
            rightBackDrive.setPower(0.4);
            leftBackDrive.setPower(0.4);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.2 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(1);
            ViperSlideL.setPower(-1);
            ViperSlideR.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.6 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <1.06 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.4 )) {
            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.6 )) {
            leftFrontDrive.setPower(.8);
            rightFrontDrive.setPower(.8);
            rightBackDrive.setPower(-.8);
            leftBackDrive.setPower(-.8);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        RightClaw.setPosition(0.4);
        LeftClaw.setPosition(0.1);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
        }
        runtime.reset();
        while (runtime.seconds() < .2) {

            ViperSlideL.setPower(1);
            ViperSlideR.setPower(1);
        }
        ViperSlideL.setPower(0.15);
        ViperSlideR.setPower(0.15);
        runtime.reset();
        while (runtime.seconds() < .3) {
            leftFrontDrive.setPower(-.5);
            rightFrontDrive.setPower(-.5);
            rightBackDrive.setPower(.5);
            leftBackDrive.setPower(.5);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.2 )) {
            leftFrontDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(-.7);
            leftBackDrive.setPower(-.7);
            ViperSlideL.setPower(.85);
            ViperSlideR.setPower(.85);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.3 )) {
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(-1);
            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.6 )) {
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.24 )) {

            ViperSlideL.setPower(-.7);
            ViperSlideR.setPower(-.7);
        }
        runtime.reset();
        ViperSlideL.setPower(.15);
        ViperSlideR.setPower(.15);
        LeftClaw.setPosition(.36);
        RightClaw.setPosition(0.22);
        while (opModeIsActive() && (runtime.seconds() <.5 )) {
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <.2 )) {

            ViperSlideL.setPower(.15);
            ViperSlideR.setPower(.15);
            leftFrontDrive.setPower(-0.4);
            rightFrontDrive.setPower(-0.4);
            rightBackDrive.setPower(0.4);
            leftBackDrive.setPower(0.4);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.2 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(1);
            ViperSlideL.setPower(-1);
            ViperSlideR.setPower(-1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <.3 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        while (opModeIsActive() && (runtime.seconds() <1 )) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            leftBackDrive.setPower(1);
        }
        runtime.reset();
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

        }

        /*
         * This OpMode illustrates the basics of TensorFlow Object Detection,
         * including Java Builder structures for specifying Vision parameters.
         *
         * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
         * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
         */
    }
// end runOpMode()









