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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot{

    public static final double RACK_POWER = .8;
    public static final double LIFT_POWER = .8;
    public static final double GRAB_POWER = .6;

    public static final double CLOSED_POSITION = .5;
    public static final double OPEN_POSITION = .73;
    public static final double UP_POSITION = .3;
    public static final double DOWN_POSITION = 1.3;

    public static final double KNOCK_SPEED = .3;
    public static final int KNOCK_TIME = 600;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor arm;
    public DcMotor rack;
    public DcMotor lift;
    public DcMotor grab;

    public Servo grip;
    public Servo read;

    public ModernRoboticsI2cColorSensor sensor;

    public Robot(HardwareMap hardwareMap)
    {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        arm  = hardwareMap.get(DcMotor.class, "arm");
        rack  = hardwareMap.get(DcMotor.class, "rack");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grab = hardwareMap.get(DcMotor.class, "grab");

        grip  = hardwareMap.get(Servo.class, "grip");
        read  = hardwareMap.get(Servo.class, "read");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        rack.setDirection(DcMotor.Direction.REVERSE);
        grab.setDirection(DcMotor.Direction.REVERSE);

        grip.setPosition(OPEN_POSITION);
        read.setPosition(UP_POSITION);

        sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "sensor");
    }

    public void knockForward() throws InterruptedException
    {
        frontLeft.setPower(KNOCK_SPEED);
        frontRight.setPower(KNOCK_SPEED);
        backLeft.setPower(KNOCK_SPEED);
        backRight.setPower(KNOCK_SPEED);

        Thread.sleep(KNOCK_TIME);

        frontLeft.setPower(-KNOCK_SPEED);
        frontRight.setPower(-KNOCK_SPEED);
        backLeft.setPower(-KNOCK_SPEED);
        backRight.setPower(-KNOCK_SPEED);

        Thread.sleep(KNOCK_TIME);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        read.setPosition(UP_POSITION);
    }

    public void knockBackward() throws InterruptedException
    {
        frontLeft.setPower(-KNOCK_SPEED);
        frontRight.setPower(-KNOCK_SPEED);
        backLeft.setPower(-KNOCK_SPEED);
        backRight.setPower(-KNOCK_SPEED);

        Thread.sleep(KNOCK_TIME);

        frontLeft.setPower(KNOCK_SPEED);
        frontRight.setPower(KNOCK_SPEED);
        backLeft.setPower(KNOCK_SPEED);
        backRight.setPower(KNOCK_SPEED);

        Thread.sleep(KNOCK_TIME);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        read.setPosition(UP_POSITION);
    }


}
