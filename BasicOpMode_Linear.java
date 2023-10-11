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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * The robot is at its fasted speed, make sure to lower the speed at some point
 */

@TeleOp(name="Robot_Control", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor lift = null;
    private CRServo arm = null;
    private Servo move = null;

    boolean open = true;
    boolean up = true;
    double pow = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");
        lift = hardwareMap.get(DcMotor.class, "liftmotor");

        arm = hardwareMap.crservo.get("Arm");
        move = hardwareMap.servo.get("rotate");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);


            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // Setup a variable for each drive wheel to save power level for telemetry
            double r = -(Math.hypot(gamepad2.left_stick_y, gamepad2.left_stick_x));
            double robotAngle = Math.atan2((gamepad2.left_stick_x), (gamepad2.left_stick_y)) + Math.PI / 4;
            double rightX = gamepad2.right_stick_x;

            lift.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);



            final double multiplier = .75;
            final double v1 = (r * Math.cos(robotAngle) + rightX) * multiplier;
            final double v2 = (r * Math.sin(-robotAngle) - rightX) * multiplier;
            final double v3 = (r * Math.sin(-robotAngle) + rightX) * multiplier;
            final double v4 = (r * Math.cos(robotAngle) - rightX) * multiplier;
            pow = move.getPosition();

            if (currentGamepad2.a && !previousGamepad2.a) {
                if (open) {
                    arm.setDirection(CRServo.Direction.REVERSE);
                    arm.setPower(-80);
                } else {
                    arm.setDirection(CRServo.Direction.REVERSE);
                    arm.setPower(80);
                }
                open = !open;
            }

            if(gamepad2.right_bumper){
                pow += 0.001;
            } else if (gamepad2.left_bumper) {
                pow -= 0.001;
            }



            frontLeftMotor.setPower(v1);
            frontRightMotor.setPower(v2);
            backLeftMotor.setPower(v3);
            backRightMotor.setPower(v4);
            move.setPosition(pow);

            final double secmutli = .75;
            lift.setPower((gamepad2.right_trigger*.50 - gamepad2.left_trigger)*secmutli);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", v1, v2, v3, v4);
            telemetry.update();
        }
    }
}


