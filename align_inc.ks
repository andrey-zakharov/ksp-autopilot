// align_inc.ks
// Библиотека: аналитическое вычисление узлов, аномалий, времени до узла.
// При вызове с onlyLib=true только загружает функции, не запускает основной код.
RUNONCEPATH("utils").
RUNONCEPATH("m_lib").
RUNONCEPATH("debug").
RUNONCEPATH("orbit_lib").
parameter onlyLib is false.
//parameter apT is 0.
//parameter peT is 0.
parameter inclT is 361.
parameter lonAscNode is 0. // longitude of ascending node, in degrees.


function countDown { parameter delay is 5.
    local warmStart is TIME:SECONDS + delay.
    until TIME:SECONDS >= warmStart {
        print " Get ready in " + disTime(warmStart - TIME:SECONDS).
        wait 1.
    }
}

function safeWarpTo { parameter mom.
    print "warping " + disTime(mom - TIME:SECONDS).
    kuniverse:timewarp:warpto(mom).
}

function inclination { parameter b1. parameter tNorm.
    return VANG(
        VCRS(b1:POSITION - b1:BODY:POSITION, b1:VELOCITY:ORBIT), tNorm).
        //VCRS(b2:POSITION - b2:BODY:POSITION, b2:VELOCITY:ORBIT)).
}

function printStatus {
    local tabPos is 14. local i is 0.
    print "true anom: " at(0, i). print ROUND(SHIP:ORBIT:TRUEANOMALY, 1) + " deg" at(tabPos, i).
}

function align_inclination {
    //parameter apT is 0.
    //parameter peT is 0.
    parameter inclT is 361.
    parameter lan is 0. // longitude of ascending node, in degrees.
    local ecc is SHIP:ORBIT:ECCENTRICITY.
    print "e: " + round(ecc, 5).
    local orbitNode is -1.
    local tNorm is V(0, 0, 0).
    if abs(inclT) < 360 {
        print "by orbit params".
         // Вектор нормали орбиты в координатах kOS - не работает, нужен reference
        //lock tNorm to V( sin(inclT)*sin(lan), cos(inclT), -sin(inclT)* cos(lan)):NORMALIZED.
        
        set tNorm to getNormalFromOrbitalElements(inclT, lan).
        set orbitNode to getNearestPhiToNormal(bodyPosOf, SHIP:VELOCITY:ORBIT, tNorm).

    } else {
        print "by target orbit".
        set tNorm to VCRS(bodyPosOf(TARGET), TARGET:VELOCITY:ORBIT):NORMALIZED.

        // Ближайший узел пересечения орбиты корабля с орбитой цели.
        // Возвращает LIST(phi_deg, sign, inclination_deg):
        //   phi_deg — угол от текущей позиции до ближайшего узла
        //   inclination_deg — угол между нормалями орбит (>0 восходящий узел, <0 нисходящий)
        set orbitNode to getNearestPhiToNormal(bodyPosOf, SHIP:VELOCITY:ORBIT, tNorm).
    }

    //showVec(VCRS(bodyPosOf, SHIP:VELOCITY:ORBIT):NORMALIZED * bodyPosOf:MAG, GREEN, "my normal", SHIP:BODY:POSITION).
    //showVec(tNorm * bodyPosOf:MAG, WHITE, "target normal", SHIP:BODY:POSITION).
    //local realNorm is VCRS(bodyPosOf(TARGET), TARGET:VELOCITY:ORBIT):NORMALIZED.
    //showVec(realNorm * unsafeBodyPos:MAG, RED, "TARGET", SHIP:BODY:POSITION).
    //print "Угол: " + ROUND(VANG(tNorm, realNorm), 4) + " deg".
    //print "Скаляр: " + ROUND(VDOT(tNorm, realNorm), 6).
    local isAN is orbitNode[1] > 0.
    local current is abs(orbitNode[1]).
    print "nearest found: ".
    local type is "AN".
    if not isAN {
        set type to "DN".
    }
    printStatus.
    local ttn to getTimeToPhi(orbitNode[0], ecc, SHIP:ORBIT:PERIOD, SHIP:ORBIT:TRUEANOMALY).
    print "time to "+type+" node: "+disTime(ttn)+" phi: "+ROUND(orbitNode[0], 1)+" deg, incl: "+ROUND(current, 1)+" deg".
    local nodeTime is TIME:SECONDS + ttn.
    local inclDeltaV is getInclDeltaV(VELOCITYAT(SHIP, nodeTime):ORBIT:MAG, current).
    local burntime is sumBurnTime(inclDeltaV).
    print "dv = " + ROUND(inclDeltaV, 1) + "m/s Tburn = " + disTime(burntime).
    local burnStart is nodeTime - burntime.
    // plans ends.

    // executing
    sas off.
    local sign is 1. if not isAN { set sign to -1. }
    lock goal to sign * VCRS(SHIP:POSITION - SHIP:BODY:POSITION, SHIP:VELOCITY:ORBIT):NORMALIZED.
    lock steering to goal.
    countDown.
    kuniverse:QUICKSAVETO("align").
    print "Game quicksaved before executing".
    execAim(STEERINGMANAGER:TARGET:FOREVECTOR).
    safeWarpTo(nodeTime - burntime - 15).
    // handling
    local thrl is 0.
    lock throttle to thrl.
    until TIME:SECONDS >= burnStart - 0.5 {
        wait 0.25.
        print round(burnStart - TIME:SECONDS) + "s remains" at (0, 0).
    }
    set WARP to 0.
    print "BURN start!".

    local incl is inclination(SHIP, tNorm).
    local dIncl is 0.
    until incl <= 0.1 or dIncl > 0 {
        set dIncl to incl.
        set incl to inclination(SHIP, tNorm).
        set dIncl to incl - dIncl.
        set thrl to MIN(1, incl / 2).
        //print "dInc=" +  at (0, 0).
        print "inc=" + round(incl, 1) + "° " + round(dIncl,2) at(0, 0).
        wait 0.1.
    }
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    unlock throttle.
    unlock steering.
    unlock goal.
    sas on.
    print "inclination: " + ROUND(inclination(SHIP, tNorm), 2) + "°".
}

if not onlyLib {
    align_inclination(inclT, lonAscNode).
}