For RCTA the Arm is at some fixed rotation R_ob from the octorotor.  The octorotor is at some some dynamic rotation from the world frame R_wo. The arm is at some rotation from its base R_ba.  Given we know the R_wo*R_ob, we get R_wb, or the rotation from world to base.  If the octorotor is in its zero or hover position the arm's base is R_wb = R_ob.  Thus:
R_wb*[0,0,9.8] = vector of gravity in arm base frame.

We're interested in R_ba. But to get R_ba we need to know what the arm's base gravity vector looks like so we can compare it to the arm's end gravity vector.  The IMU gives us R_wa naturally. To get R_ba we need R_ba = (R_wa - R_wo*R_ob) where R_ob is fixed.
