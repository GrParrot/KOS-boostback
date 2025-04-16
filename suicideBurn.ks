set steervec to ship:facing.
lock steering to steervec.
set burning to false.
clearScreen.
sas off.
parameter offset to 4.
parameter debug to false.
clearVecDraws().
wait until ship:verticalspeed<0.
until (ship:status = "landed" or ship:status="splashed"){
    set brakes to true.
    set availableAcc to (availableThrust / ship:mass - getGravity())*0.9.
    set timeToGround to -2*max(alt:radar-offset , 0)/ship:verticalSpeed.
    set targetVel to ship:velocity:surface:normalized * availableAcc * abs(timeToGround).
    if(debug){
        set targetvelvec to vecDraw(v(0,0,0),targetVel,red,"target",0.2,true,1).
    }
    print("timeToGround = "+ timeToGround) at (0,1).
    print("target velocity = "+ targetVel:mag) at(0,3).
    if(targetVel:mag < ship:airspeed or burning){
        set gear to true.
        set wantedAcc to capVecMagnitude((targetVel - ship:velocity:surface)*2 , availableAcc) + ship:up:vector * getGravity().
        set steervec to -ship:velocity:surface.
        set targetvelvec to vecDraw(v(0,0,0),steervec*5,red,"target",0.2,true,1).
        set ship:control:pilotmainthrottle to wantedAcc:mag / (availableThrust / ship:mass).
        set burning to true.
    }
    else{
        set steervec to -ship:velocity:surface.
        set ship:control:pilotmainthrottle to 0.
    }
}
print("Landed!").
set ship:control:pilotmainthrottle to 0.

function getGravity{
    return ship:body:mu / (ship:body:radius+ship:altitude)^2.
}

function capVecMagnitude{
    parameter Xvec.
    parameter mgnt.
    return Xvec * mgnt/Xvec:mag.
}