

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by dwu on 1/15/18.
 */

@Autonomous(name = "ONE5100", group = "5100")
//@Disabled
public class subOne extends coreAuto {

    @Override
    public void runOpMode(){
        waitForStart();
        runSuperMeta(1);
    }
}