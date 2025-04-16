clearScreen.
print "starting".
parameter WPname.
parameter groundoffset to 2.
parameter debug to false.
set point to waypoint(WPname).

set step to 1.
set steervec to v(0,0,0).
lock steering to steervec.

set maxAcc to 0.
set thrott to 0.
set timeGuess to 0.1.
set prevDv to v(9999,9999,9999).
set prevTime to time:seconds.

set overshootAlt to getOverShootAlt(point:position).
set mode to 0. //0 = burn, 1 = aero guidance, 2 = post-apoapsis aero glide, 3 = landing

until false{
    set deltaTime to time:seconds - prevTime.
    set prevTime to time:seconds.

    set tgt to point:position + (point:position-ship:body:position):normalized*overshootAlt.
    set iterations to 0.
    set vel to ship:velocity:surface.
    if(mode=0 or mode=1){
        set overshootAlt to getOverShootAlt(point:position).
    }
    else{
        set overshootAlt to 0.
    }

    set gravity to getGravVec().
    set midpointGrav to getMidGravVec().
    set airfactor to max(min(ship:airspeed^2/250^2 * ship:body:atm:altitudepressure(ship:altitude) , 1), 0.5).
    set timeGuess to max(timeGuess-5,0.1).
    set prevDv to v(9999,9999,9999).
    until false{
        set dv to getDeltaV(timeGuess , vel).
        set iterations to iterations + 1.
        if(dv:sqrmagnitude >= prevDv:sqrmagnitude and iterations>1){
            break.
        }
        else{
            set timeGuess to timeGuess+step.
            set prevDv to dv.
        }

    }
    set step to min(max(tgt:mag/30000 , 0.1), 1).
    set dV to dv.
    set dVmag to dV:mag.

    printAt("overshoot altitude = "+overshootAlt,0,6).
    print("step = "+step) at (0,15).
    print("TOF = "+ timeGuess) at (0,3).
    print("Airfactor = "+ airfactor) at (0,4).
    print("dV = "+ dVmag) at (0,1).

    set maxAcc to ship:availablethrust/ship:mass.
    set predictedAcc to (ship:control:pilotmainthrottle*maxAcc*ship:facing:vector + gravity).
    set aeroAcc to (ship:sensors:acc - predictedAcc). //only works when not on the ground
    
    print("Acceleration= "+ship:sensors:acc:mag) at (0,7).
    print("predicted acc ="+ predictedAcc:mag) at(0,8).
    print("Aero drag = "+ aeroAcc:mag) at (0,9).
    print("throttle = "+ thrott) at(0,12).

    if(aeroAcc:mag <2.5 and vDot(dV, (dV+ship:velocity:surface):normalized)<10 and dVmag<30){
        set mode to 1.
        printAt("Entering glide phase.",0,5).
    }
    if(mode=1 and ship:verticalspeed<0){
        set mode to 2.
    }
    //steering
    if(debug)
        set dVdraw to vecDraw(v(0,0,0),dV,red,"dv",0.3,true,1).
    
    if(mode = 1 or mode = 2){
        set airfactor to 1.
    }
    set aerobrake to false.
    if(mode = 2){
        set steervec to -(capVecMagnitude((dV*2+ship:velocity:surface*airfactor),maxAcc) - gravity).
        set sideVel to vectorExclude(-ship:up:vector , dV+ship:velocity:surface):mag.
        print("side velocity = "+sideVel) at (0,14).
        if(sideVel<20 and (dV+ship:velocity:surface):mag>150){
            set aerobrake to true.
        }

    }
    else if(mode = 3){
        set steervec to (capVecMagnitude((dV*1.5-ship:velocity:surface*airfactor),maxAcc) - gravity).
    }
    else{//mode 0
        set steervec to capVecMagnitude((dV+ship:velocity:surface*airfactor),maxAcc) - gravity.
    }
    set brakes to aerobrake.
    //mode 3 switch
    if((mode = 2 or mode = 4) and ship:airspeed/(maxAcc) > timeGuess and alt:radar<15000){
        set mode to 4.
        set ship:control:pilotmainthrottle to 0.
        runpath("0:/suicideBurn.ks",groundoffset, debug).
        break.
        //set mode to 3.
    }
    if(debug)
        set steerdraw to vecDraw(v(0,0,0),steervec,green,"steer",0.3,true,1).

    if(maxAcc>0 and (mode=0 or mode=3)){
        if(ship:airspeed<100){
            set thrott to max((dV-gravity):mag,0)/maxAcc.
        }
        else{
            set thrott to  max((dV-gravity):mag,0)/maxAcc * max(0 , cos(vectorAngle(dV,ship:facing:vector))).
        }
        set ship:control:pilotmainthrottle to thrott.
    }
    
    else{
        set ship:control:pilotmainthrottle to 0.
    }
}
function getDeltaV{
    parameter dT.
    parameter vel.
    //v = (s1 - s0 - a * t^2 /2) / t
    set dV to (tgt - midpointGrav * dT^2 /2)/dT - vel.
    return dV.
}

function getMidGravVec{
    set midpoint to tgt/2.
    return (ship:body:position-midpoint):normalized * ship:body:mu /(ship:body:radius+ship:altitude)^2.
}

function getGravVec{
    return (-ship:up:vector * ship:body:mu /(ship:body:radius+ship:altitude)^2).
}

function getOverShootAlt{
    parameter targ.//target position to calculate overshoot altitude
    return sqrt(targ:mag) * 40.
}

function capVecMagnitude{
    parameter Xvec.
    parameter mgnt.
    return Xvec * mgnt/Xvec:mag.
}

function getStopDistance{
    set acceleration to (maxAcc-ship:body:mu/ship:body:radius^2)*0.9.
    set stopdistance to max(100,(ship:verticalSpeed^2 / acceleration)+(-acceleration*ship:verticalSpeed/acceleration^2*0.5)).
    return stopdistance.
}