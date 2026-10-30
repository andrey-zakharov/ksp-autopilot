FUNCTION clamp { PARAMETER val. PARAMETER loVal. PARAMETER hiVal.
    IF val < loVal { RETURN loVal. }
    IF val > hiVal { RETURN hiVal. }
    RETURN val.
}

FUNCTION safeDiv { PARAMETER numerator. PARAMETER denominator. PARAMETER defVal.
    IF ABS(denominator) < 0.000001 { RETURN defVal. }
    RETURN numerator / denominator.
}

FUNCTION atmPressure { PARAMETER altM.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    RETURN BODY:ATM:ALTITUDEPRESSURE(altM).
}

FUNCTION dynPressure { PARAMETER altM. PARAMETER speedM.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    RETURN 0.5 * atmPressure(altM) * speedM * speedM.
}

// Угол тангажа вектора относительно горизонта
FUNCTION vectorPitch { PARAMETER vec.
    IF vec:MAG < 0.1 { RETURN 90. }
    LOCAL sinGamma IS VDOT(vec, SHIP:UP:FOREVECTOR) / vec:MAG.
    SET sinGamma TO clamp(sinGamma, -1, 1).
    RETURN ARCSIN(sinGamma).
}

FUNCTION vecPitch { PARAMETER directionVec.
    RETURN vectorPitch(directionVec).
}

FUNCTION vectorYaw { PARAMETER directionVec.
    RETURN ARCTAN2(directionVec:X, directionVec:Z).
}

FUNCTION vecYaw { PARAMETER directionVec.
    RETURN vectorYaw(directionVec).
}


FUNCTION thrustGate { PARAMETER desired. PARAMETER wantVec. PARAMETER maxErrDeg.
    IF wantVec:MAG < 0.000001 { RETURN desired. }

    LOCAL aimErr TO VANG(SHIP:FACING:FOREVECTOR, wantVec:NORMALIZED).

    IF aimErr > maxErrDeg { RETURN 0. }

    RETURN desired * clamp(COS(aimErr), 0.2, 1).
}
// --------------------------------------------------------------------------
// Мировая позиция точки поверхности в будущий момент времени
// Используется в land.ks для расчёта точки деорбита с учётом вращения тела
// --------------------------------------------------------------------------
FUNCTION surfacePosAt { PARAMETER latDeg. PARAMETER lngDeg. PARAMETER deltaSec.
    LOCAL rotRateDeg TO 360 / BODY:ROTATIONPERIOD.
    LOCAL totalLngDeg TO lngDeg + BODY:ROTATIONANGLE + rotRateDeg * deltaSec.

    LOCAL latRad TO latDeg * CONSTANT:PI / 180.
    LOCAL lngRad TO totalLngDeg * CONSTANT:PI / 180.

    LOCAL radiusM TO BODY:RADIUS + BODY:GEOPOSITIONLATLNG(latDeg, lngDeg):TERRAINHEIGHT.

    RETURN V(
        radiusM * COS(latRad) * COS(lngRad),
        radiusM * SIN(latRad),
        radiusM * COS(latRad) * SIN(lngRad)
    ).
}

FUNCTION aimGate { PARAMETER aimErr. PARAMETER maxErr. PARAMETER thr.
    IF aimErr > maxErr {
        RETURN 0.
    }
    RETURN thr * clamp(COS(aimErr), 0.2, 1).
}


FUNCTION stageHasUsablePropellant {
    LOCAL resList IS STAGE:RESOURCES.
    LOCAL lf TO 0.
    LOCAL ox TO 0.
    LOCAL sf TO 0.
    LOCAL mp TO 0.
    LOCAL xn TO 0.
    LOCAL hasLF TO FALSE.
    LOCAL hasOx TO FALSE.
    LOCAL hasSolid TO FALSE.
    LOCAL hasAir TO FALSE.
    LOCAL hasProp TO FALSE.
    FOR r IN resList {
        IF r:CAPACITY > 0.1 {
            IF r:NAME = "LiquidFuel" {
                SET hasLF TO TRUE.
                SET hasProp TO TRUE.
                SET lf TO lf + r:AMOUNT.
            } ELSE IF r:NAME = "Oxidizer" {
                SET hasOx TO TRUE.
                SET hasProp TO TRUE.
                SET ox TO ox + r:AMOUNT.
            } ELSE IF r:NAME = "SolidFuel" {
                SET hasSolid TO TRUE.
                SET hasProp TO TRUE.
                SET sf TO sf + r:AMOUNT.
            } ELSE IF r:NAME = "MonoPropellant" {
                SET hasProp TO TRUE.
                SET mp TO mp + r:AMOUNT.
            } ELSE IF r:NAME = "XenonGas" {
                SET hasProp TO TRUE.
                SET xn TO xn + r:AMOUNT.
            } ELSE IF r:NAME = "IntakeAir" {
                SET hasAir TO TRUE.
            }
        }
    }
    IF NOT hasProp { RETURN TRUE. }
    IF hasSolid AND sf > 0.1 { RETURN TRUE. }
    IF hasLF AND hasOx {
        IF lf > 0.1 AND ox > 0.1 { RETURN TRUE. }
        IF lf > 0.1 AND hasAir { RETURN TRUE. }
        RETURN FALSE.
    }
    IF hasLF { RETURN lf > 0.1. }
    IF mp > 0.1 OR xn > 0.1 { RETURN TRUE. }
    RETURN FALSE.
}



FUNCTION shouldStage {
    IF SHIP:MAXTHRUST <= 0.01 { RETURN TRUE. }
    IF NOT stageHasUsablePropellant() { RETURN TRUE. }
    RETURN FALSE.
}

// PD-регулятор для плавного управления тягой
// targetAccel - желаемое ускорение
// currentAccel - текущее измеренное ускорение (или скорость изменения)
// kp - коэффициент пропорциональности
// kd - коэффициент дифференцирования (демпфирование)
// prevError - ошибка с прошлого тика для расчета D-составляющей
// dt - время шага
FUNCTION pidThrottle { PARAMETER targetAccel. PARAMETER currentAccel. PARAMETER kp. PARAMETER kd. PARAMETER prevError. PARAMETER dt.
    LOCAL error TO targetAccel - currentAccel.
    
    // D-составляющая: скорость изменения ошибки
    LOCAL derivative TO 0.
    IF dt > 0.001 {
        SET derivative TO (error - prevError) / dt.
    }
    
    // Итоговое управляющее воздействие
    LOCAL output TO kp * error + kd * derivative.
    
    RETURN List(output, error).
}


// Плавный поворот текущего направления к целевому
FUNCTION rotateToward { PARAMETER currentRot. PARAMETER wantVec. PARAMETER rateDeg. PARAMETER dtSec.
    IF wantVec:MAG < 0.000001 { RETURN currentRot. }

    LOCAL curVec TO currentRot:FOREVECTOR.
    LOCAL wantN TO wantVec:NORMALIZED.
    LOCAL angErr TO VANG(curVec, wantN).

    IF angErr <= 0.05 { RETURN currentRot. }

    LOCAL rotAxis TO VCRS(curVec, wantN).
    IF rotAxis:MAG < 0.000001 { RETURN currentRot. }

    SET rotAxis TO rotAxis:NORMALIZED.

    LOCAL stepDeg TO MIN(angErr, MAX(0.01, rateDeg * dtSec)).
    RETURN ANGLEAXIS(stepDeg, rotAxis) * currentRot.
}

// Расчет времени разворота для деорбита (эвристика)
// Для тяжелых кораблей нужно больше времени, для легких меньше.
// Мы используем это для установки ETA ноды, а не для управления рулем.
FUNCTION estimateTurnTime { PARAMETER angleDeg. PARAMETER massKg.
    // Базовая скорость ~20 deg/s для легкой ракеты (<5 тонн)
    // Замедляется линейно с массой до ~5 deg/s для тяжелой (>50 тонн)
    LOCAL baseRate TO 20.
    LOCAL massFactor TO CLAMP(massKg / 50000, 0.25, 1.0). // 1.0 для 50кг+, 0.25 для 200т+
    LOCAL estRate TO MAX(5, baseRate * massFactor).
    
    // Добавляем запас 2 секунды на инерцию и стабилизацию
    RETURN (angleDeg / estRate) + 2.
}

// ==========================================================================
// Общие функции орбитальной механики и работы с нодами
// Используются в align_plane.ks и deorbit_phase.ks
// ==========================================================================

// Проверка конечности числа (ловит NaN и бесконечности)
FUNCTION isFiniteVal { PARAMETER inpVal.
    IF inpVal <> inpVal { RETURN FALSE. }
    IF inpVal > 1000000000000000 { RETURN FALSE. }
    IF inpVal < -1000000000000000 { RETURN FALSE. }
    RETURN TRUE.
}

// Проверка конечности вектора
FUNCTION vecIsOk { PARAMETER inpVec.
    IF NOT isFiniteVal(inpVec:MAG) { RETURN FALSE. }
    IF NOT isFiniteVal(inpVec:X) { RETURN FALSE. }
    IF NOT isFiniteVal(inpVec:Y) { RETURN FALSE. }
    IF NOT isFiniteVal(inpVec:Z) { RETURN FALSE. }
    RETURN TRUE.
}

// Нормализация угла в [0, 360)
FUNCTION normDeg360 { PARAMETER angVal.
    IF NOT isFiniteVal(angVal) { RETURN 0. }
    LOCAL normAng TO angVal - 360 * FLOOR(angVal / 360).
    IF normAng < 0 { SET normAng TO normAng + 360. }
    IF normAng >= 360 { SET normAng TO normAng - 360. }
    IF NOT isFiniteVal(normAng) { RETURN 0. }
    RETURN normAng.
}

// Нормаль орбиты по векторам позиции и скорости
FUNCTION orbitNormal { PARAMETER posArg. PARAMETER velArg.
    LOCAL hVec IS VCRS(posArg, velArg).
    IF NOT vecIsOk(hVec) OR hVec:MAG < 0.000001 {
        RETURN V(0, 1, 0).
    }
    RETURN hVec:NORMALIZED.
}

// ==========================================================================
// Полная орбитальная информация по векторам позиции и скорости.
// Возвращает List(rp, toPeSec, eHat, hHat, ok).
//   rp      — радиус перицентра
//   toPeSec — время до перицентра в секундах
//   eHat    — направление на перицентр (единичный вектор эксцентриситета)
//   hHat    — нормаль орбиты
//   ok      — TRUE если расчёт успешен
// ==========================================================================
FUNCTION orbitInfoFromState { PARAMETER posArg. PARAMETER velArg.
    LOCAL badList TO List(-1, 0, V(0, 0, 0), V(0, 1, 0), FALSE).

    LOCAL rMag TO posArg:MAG.
    LOCAL vMag TO velArg:MAG.
    LOCAL muVal TO BODY:MU.

    IF NOT isFiniteVal(rMag) OR NOT isFiniteVal(vMag) OR NOT isFiniteVal(muVal) {
        RETURN badList.
    }
    IF rMag < 1 OR vMag < 0.01 OR muVal <= 0 { RETURN badList. }

    LOCAL hVec TO VCRS(posArg, velArg).
    LOCAL hMag TO hVec:MAG.
    IF NOT vecIsOk(hVec) OR hMag < 0.000001 { RETURN badList. }
    LOCAL hHat TO hVec:NORMALIZED.

    LOCAL pSem TO hMag * hMag / muVal.
    LOCAL rHat TO posArg:NORMALIZED.
    LOCAL eVec TO VCRS(velArg, hVec) / muVal - rHat.
    LOCAL eMag TO eVec:MAG.

    IF NOT vecIsOk(eVec) OR NOT isFiniteVal(eMag) { RETURN badList. }

    IF eMag < 0.000001 {
        RETURN List(rMag, 0, rHat, hHat, TRUE).
    }
    IF eMag >= 0.995 { RETURN badList. }

    LOCAL rpVal TO pSem / (1 + eMag).
    LOCAL energy TO vMag * vMag / 2 - muVal / rMag.
    IF NOT isFiniteVal(rpVal) OR rpVal <= 0 { RETURN badList. }
    IF NOT isFiniteVal(energy) OR energy >= 0 { RETURN badList. }

    LOCAL smaVal TO -muVal / (2 * energy).
    LOCAL periodArg TO smaVal * smaVal * smaVal / muVal.
    IF NOT isFiniteVal(periodArg) OR periodArg <= 0 { RETURN badList. }
    LOCAL periodVal TO 2 * CONSTANT:PI * SQRT(periodArg).
    IF NOT isFiniteVal(periodVal) OR periodVal <= 0 { RETURN badList. }

    LOCAL eHat TO eVec:NORMALIZED.
    IF NOT vecIsOk(eHat) { RETURN badList. }

    LOCAL cosNu TO clamp(VDOT(eHat, rHat), -1, 1).
    LOCAL nuDeg TO ARCCOS(cosNu).
    IF VDOT(velArg, rHat) < 0 { SET nuDeg TO 360 - nuDeg. }

    LOCAL eFacArg TO (1 - eMag) / (1 + eMag).
    IF NOT isFiniteVal(eFacArg) { RETURN badList. }
    LOCAL eFac TO SQRT(MAX(0.000001, eFacArg)).
    LOCAL eAnDeg TO normDeg360(2 * ARCTAN2(eFac * SIN(nuDeg / 2), COS(nuDeg / 2))).
    LOCAL mDeg TO normDeg360(eAnDeg - (180 / CONSTANT:PI) * eMag * SIN(eAnDeg)).

    LOCAL toDeg TO 360 - mDeg.
    IF toDeg >= 360 { SET toDeg TO 0. }
    IF toDeg < 0 { SET toDeg TO 0. }

    LOCAL toPeSec TO toDeg / 360 * periodVal.
    IF NOT isFiniteVal(toPeSec) OR toPeSec < 0 { RETURN badList. }

    RETURN List(rpVal, toPeSec, eHat, hHat, TRUE).
}

// ==========================================================================
// Поиск времени пересечения орбиты с плоскостью (нормаль = planeNormal).
// Использует бисекцию. Возвращает время в секундах от текущего момента.
// ==========================================================================
FUNCTION findPlaneCrossTime { PARAMETER planeNormal. PARAMETER minTime. PARAMETER periodSec.
    LOCAL curUT TO TIME:SECONDS.
    LOCAL stepSec TO MAX(5, periodSec / 48).

    LOCAL timeA TO minTime.
    LOCAL projA TO VDOT(bodyPosAt(timeA), planeNormal).

    LOCAL bestTime TO timeA.
    LOCAL bestAbsVal TO ABS(projA).

    LOCAL timeB TO timeA + stepSec.
    LOCAL limitT TO minTime + periodSec + stepSec.

    UNTIL timeB > limitT {
        LOCAL posB TO bodyPosAt(timeB).
        LOCAL projB TO VDOT(posB, planeNormal).

        IF ABS(projB) < bestAbsVal {
            SET bestAbsVal TO ABS(projB).
            SET bestTime TO timeB.
        }

        IF (projA >= 0 AND projB < 0) OR (projA < 0 AND projB >= 0) {
            LOCAL lowT TO timeA.
            LOCAL highT TO timeB.
            LOCAL lowProj TO projA.
            LOCAL bisectIdx TO 0.

            UNTIL bisectIdx >= 20 OR (highT - lowT) < 0.5 {
                LOCAL midT TO (lowT + highT) / 2.
                LOCAL posRawM TO POSITIONAT(SHIP, TIMESTAMP(curUT + midT)).
                LOCAL bodyPosM TO BODY:POSITION.
                LOCAL posM TO posRawM.
                IF vecIsOk(bodyPosM) AND bodyPosM:MAG > 1 {
                    SET posM TO posRawM - bodyPosM.
                }
                LOCAL projM TO VDOT(posM, planeNormal).

                IF (lowProj >= 0 AND projM < 0) OR (lowProj < 0 AND projM >= 0) {
                    SET highT TO midT.
                } ELSE {
                    SET lowT TO midT.
                    SET lowProj TO projM.
                }
                SET bisectIdx TO bisectIdx + 1.
            }

            RETURN (lowT + highT) / 2.
        }

        SET timeA TO timeB.
        SET projA TO projB.
        SET timeB TO timeB + stepSec.
        WAIT 0.
    }

    RETURN bestTime.
}

// ==========================================================================
// Калибровка реальных осей ноды.
// Нода должна быть уже создана и добавлена (например с NORMAL=1).
// Возвращает List(realRadial, realNormal, realPrograde).
// ==========================================================================
FUNCTION calibrateNodeAxes {
    IF NOT HASNODE {
        RETURN List(V(0, 0, 0), V(0, 0, 0), V(0, 0, 0)).
    }

    SET NEXTNODE:RADIALOUT TO 0.
    SET NEXTNODE:NORMAL TO 0.
    SET NEXTNODE:PROGRADE TO 0.
    WAIT 0.1.

    SET NEXTNODE:NORMAL TO 1.
    WAIT 0.1.
    LOCAL realNormal TO NEXTNODE:DELTAV:NORMALIZED.

    SET NEXTNODE:NORMAL TO 0.
    SET NEXTNODE:PROGRADE TO 1.
    WAIT 0.1.
    LOCAL realPrograde TO NEXTNODE:DELTAV:NORMALIZED.

    SET NEXTNODE:PROGRADE TO 0.
    SET NEXTNODE:RADIALOUT TO 1.
    WAIT 0.1.
    LOCAL realRadial TO NEXTNODE:DELTAV:NORMALIZED.

    RETURN List(realRadial, realNormal, realPrograde).
}

// Позиция корабля в момент через deltaSec секунд, ОТ ЦЕНТРА ТЕЛА
FUNCTION bodyPosAt { PARAMETER deltaSec.
    LOCAL rawPos TO POSITIONAT(SHIP, TIMESTAMP(TIME:SECONDS + deltaSec)).
    LOCAL bodyPos TO BODY:POSITION.
    IF vecIsOk(bodyPos) AND bodyPos:MAG > 1 {
        RETURN rawPos - bodyPos.
    }
    RETURN rawPos.
}