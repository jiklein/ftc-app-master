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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Double_Player", group="TeleOp")
//@Disabled
public class Double_Player_Iterative_OpMode extends OpMode
{
    private Robot robot;
    private boolean grabOpen;
    private boolean pressed;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);

        pressed = true;
        grabOpen = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double power[] = new double[4];

        power[0] = gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x;
        power[1] = gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x;
        power[2] = gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x;
        power[3] = gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x;

        for (int i=0; i<power.length; i++) {
            power[i] = adj(power[i]);
            power[i] /= 2;
        }

        robot.frontLeft.setPower(power[0]);
        robot.backLeft.setPower(power[1]);
        robot.frontRight.setPower(power[2]);
        robot.backRight.setPower(power[3]);

        if (gamepad2.left_bumper)
            robot.rack.setPower(Robot.RACK_POWER);
        else if (gamepad2.right_bumper)
            robot.rack.setPower(-Robot.RACK_POWER);
        else
            robot.rack.setPower(0);

        if (gamepad2.left_trigger>0)
            robot.arm.setPower(gamepad2.left_trigger);
        else
            robot.arm.setPower(-gamepad2.right_trigger);

        if (gamepad2.x&&pressed)
        {
            if (grabOpen)
                grabOpen = false;

            else
                grabOpen = true;

            if (grabOpen)
                robot.grip.setPosition(Robot.OPEN_POSITION);

            else
                robot.grip.setPosition(Robot.CLOSED_POSITION);
            pressed = false;
        }
        else if (!(pressed || gamepad2.x))
            pressed = true;


        if (gamepad2.dpad_left)
            robot.lift.setPower(robot.LIFT_POWER);
        else if (gamepad2.dpad_right)
            robot.lift.setPower(-robot.LIFT_POWER);
        else
            robot.lift.setPower(0);

        if (gamepad2.dpad_up)
            robot.grab.setPower(Robot.GRAB_POWER);
        else if (gamepad2.dpad_down)
            robot.grab.setPower(-Robot.GRAB_POWER);
        else
            robot.grab.setPower(0);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private double adj(double d)
    {
        return Math.max(-1,Math.min(1,d));
    }
}
