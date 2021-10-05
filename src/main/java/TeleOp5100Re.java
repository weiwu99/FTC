import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pliu on 10/23/17.
 */

@TeleOp(name = "5100teleOpRe", group = "5100")
public class TeleOp5100Re extends OpMode {
    private DcMotor LeftFront, RightFront, LeftBack, RightBack,
            Lifting, Ramp;

    private Servo leftservo, rightservo;
    private float pwr;

    public static float leftX, leftY, rightX, rightY;

    public void init() {
        LeftFront = hardwareMap.dcMotor.get("LF");
        RightFront = hardwareMap.dcMotor.get("RF");
        LeftBack = hardwareMap.dcMotor.get("LB");
        RightBack = hardwareMap.dcMotor.get("RB");

        leftservo = hardwareMap.servo.get("servo1");
        rightservo = hardwareMap.servo.get("servo2");

        Lifting = hardwareMap.dcMotor.get("LS");
        Ramp = hardwareMap.dcMotor.get("RP");

    }


    public void loop(){

        getJoyval();

        //move
        pwr = rightX;

        if(gamepad1.right_trigger>0){
            LeftFront.setPower(Range.clip(pwr + leftY, -0.4, 0.4));
            LeftBack.setPower(Range.clip(pwr + leftY, -0.4, 0.4));

            RightFront.setPower(Range.clip(pwr - leftY, -0.4, 0.4));
            RightBack.setPower(Range.clip(pwr - leftY, -0.4, 0.4));
        }
        else {
            LeftFront.setPower(Range.clip(pwr + leftY, -0.8, 0.8));
            LeftBack.setPower(Range.clip(pwr + leftY, -0.8, 0.8));

            RightFront.setPower(Range.clip(pwr - leftY, -0.8, 0.8));
            RightBack.setPower(Range.clip(pwr - leftY, -0.8, 0.8));
        }

        if(gamepad1.dpad_right) {
            LeftFront.setPower(0.8);
            LeftBack.setPower(-0.8);
            RightFront.setPower(0.8);
            RightBack.setPower(-0.8);
        } else if(gamepad1.dpad_left){
            LeftFront.setPower(-0.8);
            LeftBack.setPower(0.8);
            RightFront.setPower(-0.8);
            RightBack.setPower(0.8);
        }

        if (gamepad1.dpad_down){
            LeftFront.setPower(0);
            LeftBack.setPower(-0.9);
            RightFront.setPower(0);
            RightBack.setPower(-0.9);
        } else if(gamepad1.dpad_up){
            LeftFront.setPower(0);
            LeftBack.setPower(0.9);
            RightFront.setPower(0);
            RightBack.setPower(0.9);
        }




        //lifting

        if(gamepad2.a)
            Lifting.setPower(0.6);

        else if (gamepad2.y)
            Lifting.setPower(-0.6);

        else
            Lifting.setPower(0);

        //Ramp
        if(gamepad2.right_bumper)
            Ramp.setPower(-0.6);
        else if(gamepad2.left_bumper)
            Ramp.setPower(0.6);
        else
            Ramp.setPower(0);
        //grabber
        if(gamepad2.left_trigger > 0)
            leftservo.setPosition(0.75 + gamepad2.left_trigger*0.25);

        else
            leftservo.setPosition(0.75);

        if(gamepad2.right_trigger > 0)
            rightservo.setPosition(0.25 - gamepad2.right_trigger*0.25);

        else
            rightservo.setPosition(0.25);

    }

    public void getJoyval() {
        leftY = -gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightY = gamepad1.right_stick_y;
        rightX = gamepad1.right_stick_x;
    }
}
