@lazyglobal on.
// ==========================================================================
// orbit_lib.ks — единая библиотека орбитальной механики и расчётов посадки
// ==========================================================================
RUNONCEPATH("utils").

// --------------------------------------------------------------------------
// ОРБИТАЛЬНАЯ МЕХАНИКА
// --------------------------------------------------------------------------

// Истинная аномалия (град) -> эксцентрическая (град)
function convertTAtoEA { parameter ta. parameter ecc.
    return 2 * ARCTAN2(SQRT(1 - ecc) * SIN(ta / 2), SQRT(1 + ecc) * COS(ta / 2)).
}

// Эксцентрическая аномалия (град) -> средняя (град)
function convertEAtoMA { parameter ea. parameter ecc.
    return ea - ecc * SIN(ea) * (180 / CONSTANT:PI).
}

// Истинная аномалия (град) -> средняя аномалия (град)
function getMA { parameter trueanom. parameter ecc.
    return convertEAtoMA(convertTAtoEA(trueanom, ecc), ecc).
}

// Время до узла по углу phi (град) от текущей истинной аномалии
function getTimeToPhi { parameter phiDeg. parameter ecc. parameter period. parameter curNu.
    local m0 to getMA(curNu, ecc).
    local m1 to getMA(curNu + phiDeg, ecc).
    local dM to m1 - m0.
    until dM > 0 { set dM to dM + 360. }
    return dM / (360 / MAX(1, period)).
}

// dV для смены наклонения
function getInclDeltaV { parameter orbvel. parameter deltaincl.
    return 2 * orbvel * SIN(deltaincl / 2).
}

// Ближайший узел пересечения орбиты с произвольной нормалью
// Возвращает LIST(phi_deg, inclination_deg)
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

// Острая нормаль (выбирает направление с углом < 90°)
function acuteNormal { parameter refN. parameter tgtN.
    if not vecIsOk(tgtN) or tgtN:MAG < 0.5 { return refN:NORMALIZED. }
    if VANG(refN:NORMALIZED, tgtN:NORMALIZED) > 90 { return (-1) * tgtN:NORMALIZED. }
    return tgtN:NORMALIZED.
}

// Нормаль орбиты по наклонению и LAN (правильная формула с минусом)
function getNormalFromOrbitalElements { parameter incDeg. parameter lanDeg.
    local nAxis to BODY:ANGULARVEL:NORMALIZED.
    local rDir to SOLARPRIMEVECTOR - (VDOT(SOLARPRIMEVECTOR, nAxis) * nAxis).
    if rDir:MAG < 0.000001 {
        set rDir to V(1, 0, 0).
        if ABS(VDOT(rDir, nAxis)) > 0.9 { set rDir to V(0, 0, 1). }
        set rDir to rDir - (VDOT(rDir, nAxis) * nAxis).
    }
    set rDir to rDir:NORMALIZED.
    local pDir to VCRS(rDir, nAxis):NORMALIZED.
    return (COS(incDeg) * nAxis -
            SIN(incDeg) * ((SIN(lanDeg) * rDir) + (COS(lanDeg) * pDir))):NORMALIZED.
}

// Позиция объекта относительно центра тела
function bodyPosOf { parameter bd is SHIP.
    return POSITIONAT(bd, TIMESTAMP(TIME:SECONDS)) - bd:BODY:POSITION.
}

// --------------------------------------------------------------------------
// ГЕОМЕТРИЯ ЦЕЛИ С УЧЁТОМ ВРАЩЕНИЯ ТЕЛА
// --------------------------------------------------------------------------

// Позиция точки поверхности в будущий момент времени
function targetVecAtTime { parameter latDeg. parameter lngDeg. parameter terrM. parameter deltaSec.
    local geoPoint to BODY:GEOPOSITIONLATLNG(latDeg, lngDeg).
    local baseVec to geoPoint:POSITION - BODY:POSITION.
    if not vecIsOk(baseVec) or baseVec:MAG < 1 { return V(0, 0, 0). }
    local dirVec to baseVec:NORMALIZED.
    local angVelVec to BODY:ANGULARVEL.
    if vecIsOk(angVelVec) and angVelVec:MAG > 0.00000001 {
        local rotAngleDeg to angVelVec:MAG * deltaSec * (180 / CONSTANT:PI).
        set rotAngleDeg to normDeg360(rotAngleDeg).
        set dirVec to ANGLEAXIS(rotAngleDeg, angVelVec:NORMALIZED) * dirVec.
    }
    return (BODY:RADIUS + terrM) * dirVec.
}

// Нормаль целевой плоскости (для смены наклонения)
function targetNormalAtTime { parameter latDeg. parameter lngDeg. parameter terrM. parameter deltaSec is 0.
    local tgtVec to targetVecAtTime(latDeg, lngDeg, terrM, deltaSec).
    if not vecIsOk(tgtVec) or tgtVec:MAG < 1 { return V(0, 0, 0). }
    local tgtHat to tgtVec:NORMALIZED.
    local spinAxis to BODY:ANGULARVEL.
    if not vecIsOk(spinAxis) or spinAxis:MAG < 0.00000001 { return V(0, 1, 0). }
    set spinAxis to spinAxis:NORMALIZED.
    local spinProj to spinAxis - (VDOT(spinAxis, tgtHat) * tgtHat).
    if not vecIsOk(spinProj) or spinProj:MAG < 0.000001 { return V(0, 0, 0). }
    return spinProj:NORMALIZED.
}

// --------------------------------------------------------------------------
// РАСЧЁТЫ ПОСАДКИ
// --------------------------------------------------------------------------

// Дистанция торможения (высота суицид-бёрна)
function calcBurnDistance { parameter tiltDeg is 0. parameter tSpeed is 0.
    local sbMass to MAX(0.001, SHIP:MASS).
    local sbMaxThr to SHIP:MAXTHRUST.
    if sbMaxThr <= 0.01 { set sbMaxThr to SHIP:AVAILABLETHRUST. }
    local sbMaxAcc to MAX(0.01, sbMaxThr / sbMass).
    local sbG to safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).
    local sbVVertAbs to ABS(VDOT(SHIP:VELOCITY:SURFACE, SHIP:UP:FOREVECTOR)).
    local sbNetMax to MAX(0.01, sbMaxAcc * COS(tiltDeg) - sbG).
    if sbNetMax <= 0.001 { return 999999. }
    local dist to (sbVVertAbs * sbVVertAbs - tSpeed * tSpeed) / (2 * sbNetMax).
    if dist < 0 { return 0. }
    return dist.
}

// Время свободного падения с высоты (без начальной скорости)
function estimateFallTime { parameter posVec.
    local altNow to MAX(0, posVec:MAG - BODY:RADIUS).
    if altNow < 10 { return 0. }
    local gNow to safeDiv(BODY:MU, posVec:MAG * posVec:MAG, 0.001).
    if gNow <= 0.000001 { return 0. }
    return SQRT(2 * altNow / gNow).
}

// Время свободного падения с учётом начальной вертикальной скорости
function getFreefallTimeWithV { parameter altNow. parameter brakeDist. parameter vVertNow.
    local sbG to safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).
    if sbG <= 0.000001 { return 9999. }
    local vAbs to ABS(vVertNow).
    local effAlt to MAX(0, altNow - brakeDist).
    if effAlt <= 0 { return 0. }
    local disc to vAbs * vAbs + 2 * sbG * effAlt.
    if disc < 0 { return 0. }
    return (SQRT(disc) - vAbs) / sbG.
}

// Угловая скорость вращения тела с учётом направления орбиты
function bodyRotRate { parameter posVec. parameter velVec.
    local rotPeriod to BODY:ROTATIONPERIOD.
    if not isFiniteVal(rotPeriod) or rotPeriod <= 1 { return 0. }
    local rotRate to 360 / rotPeriod.
    local hVec to VCRS(posVec, velVec).
    local spinAxis to BODY:ANGULARVEL.
    if vecIsOk(hVec) and vecIsOk(spinAxis) and
       hVec:MAG > 0.000001 and spinAxis:MAG > 0.000001 {
        if VDOT(hVec:NORMALIZED, spinAxis:NORMALIZED) < 0 {
            set rotRate to (-1) * rotRate.
        }
    }
    return rotRate.
}