// align_inc.ks
// Библиотека: аналитическое вычисление узлов, аномалий, времени до узла.
// При вызове с onlyLib=true только загружает функции, не запускает основной код.
RUNONCEPATH("utils").
RUNONCEPATH("m_lib").
parameter onlyLib is false.
//parameter apT is 0.
//parameter peT is 0.
parameter inclT is 0.
parameter lonAscNode is 0. // longitude of ascending node, in degrees.

function unsafeBodyPos { parameter bd is SHIP.
    return POSITIONAT(bd, TIMESTAMP(TIME:SECONDS)) - bd:BODY:POSITION.
}

function showVec { parameter drawVec. parameter color is WHITE. parameter label is "". parameter origin is SHIP:POSITION.
    local v1 is VECDRAW(origin, drawVec, color, label).
    set v1:show to true.
    return v1.
}


// Ближайший узел пересечения орбиты с произвольной нормалью.
// Аргументы: позиция корабля от центра тела, орбитальная скорость, целевая нормаль.
// Возвращает LIST(phi_deg, sign, inclination_deg) — аналогично getNearestPhi.
function getNearestPhiToNormal { parameter pos. parameter vel. parameter tgtNorm.
    local myNorm to VCRS(pos, vel):NORMALIZED.
    local nUse to tgtNorm:NORMALIZED.

    local nodesVec to VCRS(myNorm, nUse):NORMALIZED.
    local anphi to ARCCOS(clamp(VDOT(pos:NORMALIZED, nodesVec), -1, 1)).

    if VDOT(VCRS(pos, nodesVec):NORMALIZED, myNorm) < 0 {
        set anphi to 360 - anphi.
    }
    local dnphi to MOD(anphi + 180, 360).
    local incDeg to VANG(myNorm, nUse).

    if anphi <= dnphi {
        return LIST(anphi, incDeg).
    } else {
        return LIST(dnphi, -incDeg).
    }
}

// Истинная аномалия (град) -> эксцентрическая (град)
function convertTAtoEA { parameter ta. parameter ecc.
    return 2 * ARCTAN2(SQRT(1 - ecc) * SIN(ta / 2), SQRT(1 + ecc) * COS(ta / 2)).
}

// Эксцентрическая аномалия (град) -> средняя (град)
// Формула Кеплера M = E - e*sin(E) работает в радианах,
// поэтому e*sin(E) умножается на 180/PI для перевода в градусы.
function convertEAtoMA { parameter ea. parameter ecc.
    return ea - ecc * SIN(ea) * (180 / CONSTANT:PI).
}

// Истинная аномалия (град) -> средняя аномалия (град)
function getMA { parameter trueanom. parameter ecc.
    return convertEAtoMA(convertTAtoEA(trueanom, ecc), ecc).
}

// Время до узла по углу phi (град) от текущей истинной аномалии.
// Возвращает секунды.
function getTimeToPhi { parameter phiDeg. parameter ecc. parameter period. parameter curNu.
    local m0 to getMA(curNu, ecc).
    local m1 to getMA(curNu + phiDeg, ecc).
    local dM to m1 - m0.
    until dM > 0 { set dM to dM + 360. }
    //print "dM: " + ROUND(deltaM, 1).
    return dM / (360 / MAX(1, period)).
}

// dV для смены наклонения на given скорости
function getInclDeltaV { parameter orbvel. parameter deltaincl.
    return 2 * orbvel * SIN(deltaincl / 2).
}

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

// ==========================================================================
// Вычисление нормали орбиты по наклонению (inc) и долготе восходящего узла (LAN).
// Использует SOLARPRIMEVECTOR как абсолютное опорное направление.
// ==========================================================================
function getNormalFromOrbitalElements { parameter incDeg. parameter lanDeg.
    local refDir to SOLARPRIMEVECTOR.
    local bodyNorth to BODY:ANGULARVEL:NORMALIZED.
    local refProj to refDir - (VDOT(refDir, bodyNorth) * bodyNorth).
    // Защита от вырожденного случая (ось вращения совпадает с опорным вектором)
    if refProj:MAG < 0.000001 {
        local fallback to V(1, 0, 0).
        if ABS(VDOT(fallback, bodyNorth)) > 0.9 { set fallback to V(0, 0, 1). }
        set refProj to fallback - (VDOT(fallback, bodyNorth) * bodyNorth).
    }
    set refProj to refProj:NORMALIZED.
    local nodeVec to ANGLEAXIS(lanDeg, bodyNorth) * refProj.
    local normalVec to ANGLEAXIS(incDeg, nodeVec) * bodyNorth.
    return normalVec:NORMALIZED.
}

function align_inclination {
    //parameter apT is 0.
    //parameter peT is 0.
    parameter inclT is 361.
    parameter lan is 0. // longitude of ascending node, in degrees.
    local ecc is SHIP:ORBIT:ECCENTRICITY.
    print "e: " + round(ecc, 5).
    local orbitNode is -1.
    if abs(inclT) < 360 {
        print "by orbit params".
         // Вектор нормали орбиты в координатах kOS - не работает, нужен reference
        //lock tNorm to V( sin(inclT)*sin(lan), cos(inclT), -sin(inclT)* cos(lan)):NORMALIZED.
        
        lock tNorm to getNormalFromOrbitalElements(inclT, lan).
        set orbitNode to getNearestPhiToNormal(unsafeBodyPos, SHIP:VELOCITY:ORBIT, tNorm).

    } else {
        print "by target orbit".
        lock tNorm to VCRS(unsafeBodyPos(TARGET), TARGET:VELOCITY:ORBIT):NORMALIZED.

        // Ближайший узел пересечения орбиты корабля с орбитой цели.
        // Возвращает LIST(phi_deg, sign, inclination_deg):
        //   phi_deg — угол от текущей позиции до ближайшего узла
        //   inclination_deg — угол между нормалями орбит (>0 восходящий узел, <0 нисходящий)
        set orbitNode to getNearestPhiToNormal(unsafeBodyPos, SHIP:VELOCITY:ORBIT, tNorm).
    }

    showVec(VCRS(unsafeBodyPos, SHIP:VELOCITY:ORBIT):NORMALIZED * unsafeBodyPos:MAG, GREEN, "my normal", SHIP:BODY:POSITION).
    showVec(tNorm * unsafeBodyPos:MAG, WHITE, "target normal", SHIP:BODY:POSITION).
    
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
    lock aimV to sign * VCRS(SHIP:POSITION - SHIP:BODY:POSITION, SHIP:VELOCITY:ORBIT):NORMALIZED.
    lock steering to aimV.
    countDown.
    kuniverse:quicksave.
    print "Game quicksaved before executing".
    execAim(STEERINGMANAGER:TARGET:FOREVECTOR).
    safeWarpTo(nodeTime - burntime - 15).
    // handling
    local thrl is 0.
    lock throttle to thrl.
    until TIME:SECONDS >= burnStart - 0.5 {
        wait 0.25.
    }
    set WARP to 0.
    print "BURN start!".

    local incl is inclination(SHIP, tNorm).
    local dIncl is 0.
    until incl <= 0.1 or dIncl > 0.1 {
        set dIncl to incl.
        set incl to inclination(SHIP, tNorm).
        set dIncl to incl - dIncl.
        set thrl to MIN(1, incl / 2).
        //print "dInc=" +  at (0, 0).
        print "inc=" + round(incl, 1) + "° " + round(dIncl,2).
        wait 0.1.
    }
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    unlock throttle.
    unlock steering.
    sas on.
    print "inclination: " + ROUND(inclination(SHIP, tNorm), 2) + "°".
}

if not onlyLib {
    align_inclination(inclT, lonAscNode).
}