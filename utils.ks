@lazyglobal off.

// ==========================================================================
// utils.ks — общие функции проекта
// ==========================================================================

// --------------------------------------------------------------------------
// Удаление всех нод
// --------------------------------------------------------------------------
function removeNodes {
    until not HASNODE {
        remove NEXTNODE.
        wait 0.
    }
}

// --------------------------------------------------------------------------
// Проверки чисел и векторов
// --------------------------------------------------------------------------
function isFiniteVal { parameter valueNum.
    if valueNum <> valueNum { return false. }
    if valueNum > 1000000000000000 { return false. }
    if valueNum < -1000000000000000 { return false. }
    return true.
}

function vecIsOk { parameter vecValue.
    if not isFiniteVal(vecValue:MAG) { return false. }
    if not isFiniteVal(vecValue:X) { return false. }
    if not isFiniteVal(vecValue:Y) { return false. }
    if not isFiniteVal(vecValue:Z) { return false. }
    return true.
}

// --------------------------------------------------------------------------
// Ограничение значения
// --------------------------------------------------------------------------
function clamp { parameter valueNum. parameter loNum. parameter hiNum.
    if valueNum < loNum { return loNum. }
    if valueNum > hiNum { return hiNum. }
    return valueNum.
}

// Алиас для совместимости со старыми вызовами
function clampVal { parameter valueNum. parameter loNum. parameter hiNum.
    return clamp(valueNum, loNum, hiNum).
}

// --------------------------------------------------------------------------
// Безопасное деление
// --------------------------------------------------------------------------
function safeDiv { parameter numValue. parameter denValue. parameter fallbackValue.
    if not isFiniteVal(numValue) { return fallbackValue. }
    if not isFiniteVal(denValue) { return fallbackValue. }
    if ABS(denValue) < 0.000001 { return fallbackValue. }

    local resValue to numValue / denValue.
    if not isFiniteVal(resValue) { return fallbackValue. }
    return resValue.
}

// --------------------------------------------------------------------------
// Нормализация углов
// --------------------------------------------------------------------------
function normDeg360 { parameter angValue.
    if not isFiniteVal(angValue) { return 0. }

    local normValue to angValue - 360 * FLOOR(angValue / 360).
    if normValue < 0 { set normValue to normValue + 360. }
    if normValue >= 360 { set normValue to normValue - 360. }
    if not isFiniteVal(normValue) { return 0. }

    return normValue.
}

function wrap180Deg { parameter angValue.
    return normDeg360(angValue + 180) - 180.
}

// Знаковый угол от fromVec к toVec относительно нормаль.
// Положительный = по направлению нормали, отрицательный = против.
function signedAngleDeg { parameter fromVec. parameter toVec. parameter normalVec.
    local angValue to VANG(fromVec, toVec).
    if not isFiniteVal(angValue) { return 999. }

    local crossVec to VCRS(fromVec, toVec).
    if vecIsOk(crossVec) and VDOT(crossVec, normalVec) < 0 {
        set angValue to (-1) * angValue.
    }

    return wrap180Deg(angValue).
}

// --------------------------------------------------------------------------
// Оценка времени разворота (эвристика для планирования нод)
// --------------------------------------------------------------------------
function estimateTurnTime { parameter angleDeg. parameter massValue.
    local absDeg to ABS(angleDeg).
    if not isFiniteVal(absDeg) or absDeg < 0.1 { return 0. }

    local massTons to 1.
    if isFiniteVal(massValue) {
        set massTons to clamp(massValue, 0.05, 1000).
    }

    local rateDeg to 25 / SQRT(massTons).
    set rateDeg to clamp(rateDeg, 2, 35).

    local timeValue to absDeg / rateDeg + 1.5.
    if not isFiniteVal(timeValue) { return 5. }

    return MAX(1, timeValue).
}

function safeLookDirUp { parameter dirV. parameter upV.
    local fV to dirV.
    local uV to upV.

    if not vecIsOk(fV) or fV:MAG < 0.000001 {
        set fV to uV.
    } else {
        set fV to fV:NORMALIZED.
    }

    if not vecIsOk(uV) or uV:MAG < 0.000001 {
        set uV to V(0, 1, 0).
    } else {
        set uV to uV:NORMALIZED.
    }

    if not vecIsOk(fV) or fV:MAG < 0.000001 {
        set fV to V(0, 0, 1).
    }

    if ABS(VDOT(fV, uV)) > 0.9999 {
        local sV to VCRS(fV, V(0, 1, 0)).

        if not vecIsOk(sV) or sV:MAG < 0.000001 {
            set sV to VCRS(fV, V(1, 0, 0)).
        }

        if vecIsOk(sV) and sV:MAG > 0.000001 {
            set uV to (uV + sV:NORMALIZED * 0.02):NORMALIZED.
        } else {
            set uV to V(0, 1, 0).
        }
    }

    return LOOKDIRUP(fV, uV).
}

// --------------------------------------------------------------------------
// Плавный поворот направления к целевому вектору
// --------------------------------------------------------------------------
function rotateToward { parameter currentFacing. parameter targetVec. parameter maxDeg. parameter dtSec.
    local curDir to currentFacing:FOREVECTOR.
    local tgtDir to targetVec.
    local upVec to SHIP:UP:FOREVECTOR.

    if not vecIsOk(tgtDir) or tgtDir:MAG < 0.000001 { return currentFacing. }
    set tgtDir to tgtDir:NORMALIZED.

    if not vecIsOk(curDir) or curDir:MAG < 0.000001 {
        return LOOKDIRUP(tgtDir, upVec).
    }
    set curDir to curDir:NORMALIZED.

    local angValue to VANG(curDir, tgtDir).
    if not isFiniteVal(angValue) { return currentFacing. }
    if angValue < 0.05 { return currentFacing. }

    local stepValue to 5.
    if isFiniteVal(maxDeg) { set stepValue to maxDeg. }
    if isFiniteVal(dtSec) and dtSec > 0.001 { set stepValue to stepValue * dtSec. }
    set stepValue to clamp(stepValue, 0.1, 180).

    if stepValue >= angValue { return LOOKDIRUP(tgtDir, upVec). }

    local axisVec to VCRS(curDir, tgtDir).
    if not vecIsOk(axisVec) or axisVec:MAG < 0.000001 {
        return LOOKDIRUP(tgtDir, upVec).
    }
    set axisVec to axisVec:NORMALIZED.

    local newDir to ANGLEAXIS(stepValue, axisVec) * curDir.
    if not vecIsOk(newDir) or newDir:MAG < 0.000001 {
        return LOOKDIRUP(tgtDir, upVec).
    }

    return LOOKDIRUP(newDir:NORMALIZED, upVec).
}

// --------------------------------------------------------------------------
// Гейт тяги по ориентации
// --------------------------------------------------------------------------
function thrustGate { parameter desired. parameter wantVec. parameter maxErrDeg.
    if wantVec:MAG < 0.000001 { return desired. }

    local aimErr to VANG(SHIP:FACING:FOREVECTOR, wantVec:NORMALIZED).
    if aimErr > maxErrDeg { return 0. }

    return desired * clamp(COS(aimErr), 0.2, 1).
}

// --------------------------------------------------------------------------
// Позиция корабля в момент +deltaSec секунд, ОТ ЦЕНТРА ТЕЛА
// --------------------------------------------------------------------------
function bodyPosAt { parameter deltaSec.
    local dtValue to 0.
    if isFiniteVal(deltaSec) { set dtValue to deltaSec. }

    local rawPos to POSITIONAT(SHIP, TIMESTAMP(TIME:SECONDS + dtValue)).
    local bodyPos to BODY:POSITION.

    if vecIsOk(bodyPos) and bodyPos:MAG > 1 {
        return rawPos - bodyPos.
    }
    return rawPos.
}

// --------------------------------------------------------------------------
// Нормаль орбиты по векторам позиции и скорости
// --------------------------------------------------------------------------
function orbitNormal { parameter posArg. parameter velArg.
    local hVec to VCRS(posArg, velArg).
    if not vecIsOk(hVec) or hVec:MAG < 0.000001 { return V(0, 1, 0). }
    return hVec:NORMALIZED.
}

// --------------------------------------------------------------------------
// Полная орбитальная информация по векторам позиции и скорости.
// Возвращает List(rp, toPeSec, eHat, hHat, ok).
//   rp      — радиус перицентра
//   toPeSec — время до перицентра в секундах
//   eHat    — направление на перицентр (вектор эксцентриситета)
//   hHat    — нормаль орбиты
//   ok      — true если расчёт успешен
// --------------------------------------------------------------------------
function orbitInfoFromStateMu { parameter posArg. parameter velArg. parameter muVal.
    local badList to LIST(-1, 0, V(0, 0, 0), V(0, 1, 0), false).

    local rMag to posArg:MAG.
    local vMag to velArg:MAG.

    if not isFiniteVal(rMag) or not isFiniteVal(vMag) or not isFiniteVal(muVal) { return badList. }
    if rMag < 1 or vMag < 0.01 or muVal <= 0 { return badList. }

    local hVec to VCRS(posArg, velArg).
    local hMag to hVec:MAG.
    if not vecIsOk(hVec) or hMag < 0.000001 { return badList. }

    local hHat to hVec:NORMALIZED.
    local pSem to hMag * hMag / muVal.
    local rHat to posArg:NORMALIZED.
    if not vecIsOk(rHat) { return badList. }

    local eVec to VCRS(velArg, hVec) / muVal - rHat.
    local eMag to eVec:MAG.
    if not vecIsOk(eVec) or not isFiniteVal(eMag) { return badList. }

    if eMag < 0.000001 { return LIST(rMag, 0, rHat, hHat, true). }
    if eMag >= 0.995 { return badList. }

    local rpVal to pSem / (1 + eMag).
    local energy to vMag * vMag / 2 - muVal / rMag.

    if not isFiniteVal(rpVal) or rpVal <= 0 { return badList. }
    if not isFiniteVal(energy) or energy >= 0 { return badList. }

    local smaVal to -muVal / (2 * energy).
    local periodArg to smaVal * smaVal * smaVal / muVal.
    if not isFiniteVal(periodArg) or periodArg <= 0 { return badList. }

    local periodVal to 2 * CONSTANT:PI * SQRT(periodArg).
    if not isFiniteVal(periodVal) or periodVal <= 0 { return badList. }

    local eHat to eVec:NORMALIZED.
    if not vecIsOk(eHat) { return badList. }

    local cosNu to clamp(VDOT(eHat, rHat), -1, 1).
    if not isFiniteVal(cosNu) { return badList. }

    local nuDeg to ARCCOS(cosNu).
    if not isFiniteVal(nuDeg) { return badList. }

    if VDOT(velArg, rHat) < 0 { set nuDeg to 360 - nuDeg. }
    set nuDeg to normDeg360(nuDeg).

    local eFacArg to (1 - eMag) / (1 + eMag).
    if not isFiniteVal(eFacArg) { return badList. }

    local eFac to SQRT(MAX(0.000001, eFacArg)).
    if not isFiniteVal(eFac) { return badList. }

    local eAnDeg to 2 * ARCTAN2(eFac * SIN(nuDeg / 2), COS(nuDeg / 2)).
    set eAnDeg to normDeg360(eAnDeg).

    local mDeg to eAnDeg - (180 / CONSTANT:PI) * eMag * SIN(eAnDeg).
    set mDeg to normDeg360(mDeg).

    local toDeg to 360 - mDeg.
    if toDeg >= 360 { set toDeg to 0. }
    if toDeg < 0 { set toDeg to 0. }

    local toPeSec to toDeg / 360 * periodVal.
    if not isFiniteVal(toPeSec) or toPeSec < 0 { return badList. }

    return LIST(rpVal, toPeSec, eHat, hHat, true).
}

// Обёртка с текущим телом
function orbitInfoFromState { parameter posArg. parameter velArg.
    return orbitInfoFromStateMu(posArg, velArg, BODY:MU).
}

// --------------------------------------------------------------------------
// Поиск времени пересечения орбиты с плоскостью (нормаль = planeNormal).
// Возвращает время в секундах от текущего момента.
// --------------------------------------------------------------------------
function findPlaneCrossTime { parameter planeNormal. parameter minTime. parameter periodSec.
    if not vecIsOk(planeNormal) or planeNormal:MAG < 0.000001 { return -1. }
    set planeNormal to planeNormal:NORMALIZED.

    local periodValue to MAX(60, periodSec).
    if not isFiniteVal(periodValue) { set periodValue to 3600. }

    local stepValue to MAX(2, periodValue / 48).
    local timeA to MAX(0, minTime).

    local posA to bodyPosAt(timeA).
    if not vecIsOk(posA) { return -1. }

    local projA to VDOT(posA, planeNormal).
    if not isFiniteVal(projA) { return -1. }

    local bestTime to timeA.
    local bestAbs to ABS(projA).

    local timeB to timeA + stepValue.
    local limitT to timeA + periodValue + stepValue.
    local waitCnt to 0.

    until timeB > limitT {
        local posB to bodyPosAt(timeB).
        local projB to 999.

        if vecIsOk(posB) { set projB to VDOT(posB, planeNormal). }

        if isFiniteVal(projB) {
            if ABS(projB) < bestAbs {
                set bestAbs to ABS(projB).
                set bestTime to timeB.
            }

            if (projA >= 0 and projB < 0) or (projA < 0 and projB >= 0) {
                local lowT to timeA.
                local highT to timeB.
                local lowProj to projA.
                local bisectIdx to 0.

                until bisectIdx >= 22 or (highT - lowT) < 0.5 {
                    local midT to (lowT + highT) / 2.
                    local posM to bodyPosAt(midT).
                    local projM to 999.

                    if vecIsOk(posM) { set projM to VDOT(posM, planeNormal). }

                    if not isFiniteVal(projM) {
                        set highT to midT.
                    } else if (lowProj >= 0 and projM < 0) or (lowProj < 0 and projM >= 0) {
                        set highT to midT.
                    } else {
                        set lowT to midT.
                        set lowProj to projM.
                    }

                    set bisectIdx to bisectIdx + 1.
                }

                return (lowT + highT) / 2.
            }

            set timeA to timeB.
            set projA to projB.
        }

        set timeB to timeB + stepValue.
        set waitCnt to waitCnt + 1.

        if waitCnt >= 8 {
            set waitCnt to 0.
            wait 0.
        }
    }

    return bestTime.
}

// --------------------------------------------------------------------------
// Калибровка реальных осей ноды.
// Нода должна быть уже создана и добавлена.
// Возвращает List(realRadial, realNormal, realPrograde).
// --------------------------------------------------------------------------
function calibrateNodeAxes {
    local zeroList to LIST(V(0, 0, 0), V(0, 0, 0), V(0, 0, 0)).
    if not HASNODE { return zeroList. }

    set NEXTNODE:RADIALOUT to 0.
    set NEXTNODE:NORMAL to 0.
    set NEXTNODE:PROGRADE to 0.
    wait 0.05.

    set NEXTNODE:NORMAL to 1.
    wait 0.05.

    local realNormal to NEXTNODE:DELTAV.
    if not vecIsOk(realNormal) or realNormal:MAG < 0.000001 { return zeroList. }
    set realNormal to realNormal:NORMALIZED.

    set NEXTNODE:NORMAL to 0.
    set NEXTNODE:PROGRADE to 1.
    wait 0.05.

    local realPrograde to NEXTNODE:DELTAV.
    if not vecIsOk(realPrograde) or realPrograde:MAG < 0.000001 { return zeroList. }
    set realPrograde to realPrograde:NORMALIZED.

    set NEXTNODE:PROGRADE to 0.
    set NEXTNODE:RADIALOUT to 1.
    wait 0.05.

    local realRadial to NEXTNODE:DELTAV.
    if not vecIsOk(realRadial) or realRadial:MAG < 0.000001 { return zeroList. }
    set realRadial to realRadial:NORMALIZED.

    return LIST(realRadial, realNormal, realPrograde).
}

// --------------------------------------------------------------------------
// Мировая позиция точки поверхности в будущий момент времени
// --------------------------------------------------------------------------
function surfacePosAt { parameter latDeg. parameter lngDeg. parameter deltaSec.
    local rotRateDeg to 360 / BODY:ROTATIONPERIOD.
    local totalLngDeg to lngDeg + BODY:ROTATIONANGLE + rotRateDeg * deltaSec.
    local latRad to latDeg * CONSTANT:PI / 180.
    local lngRad to totalLngDeg * CONSTANT:PI / 180.
    local radiusM to BODY:RADIUS + BODY:GEOPOSITIONLATLNG(latDeg, lngDeg):TERRAINHEIGHT.

    return V(
        radiusM * COS(latRad) * COS(lngRad),
        radiusM * SIN(latRad),
        radiusM * COS(latRad) * SIN(lngRad)
    ).
}

// --------------------------------------------------------------------------
// Атмосфера и динамическое давление
// --------------------------------------------------------------------------
function atmPressure { parameter altM.
    if not BODY:ATM:EXISTS { return 0. }
    return BODY:ATM:ALTITUDEPRESSURE(altM).
}

function dynPressure { parameter altM. parameter speedM.
    if not BODY:ATM:EXISTS { return 0. }
    return 0.5 * atmPressure(altM) * speedM * speedM.
}

// --------------------------------------------------------------------------
// Углы тангажа и рыскания вектора
// --------------------------------------------------------------------------
function vectorPitch { parameter vec.
    if vec:MAG < 0.1 { return 90. }

    local sinGamma to VDOT(vec, SHIP:UP:FOREVECTOR) / vec:MAG.
    set sinGamma to clamp(sinGamma, -1, 1).
    return ARCSIN(sinGamma).
}

function vecPitch { parameter directionVec.
    return vectorPitch(directionVec).
}

function vectorYaw { parameter directionVec.
    return ARCTAN2(directionVec:X, directionVec:Z).
}

function vecYaw { parameter directionVec.
    return vectorYaw(directionVec).
}

function aimGate { parameter aimErr. parameter maxErr. parameter thr.
    if aimErr > maxErr { return 0. }
    return thr * clamp(COS(aimErr), 0.2, 1).
}

// --------------------------------------------------------------------------
// Проверка топлива и ступени
// --------------------------------------------------------------------------
function stageHasUsablePropellant {
    local resList to STAGE:RESOURCES.
    local lf to 0.
    local ox to 0.
    local sf to 0.
    local mp to 0.
    local xn to 0.
    local hasLF to false.
    local hasOx to false.
    local hasSolid to false.
    local hasAir to false.
    local hasProp to false.

    for r in resList {
        if r:CAPACITY > 0.1 {
            if r:NAME = "LiquidFuel" {
                set hasLF to true.
                set hasProp to true.
                set lf to lf + r:AMOUNT.
            } else if r:NAME = "Oxidizer" {
                set hasOx to true.
                set hasProp to true.
                set ox to ox + r:AMOUNT.
            } else if r:NAME = "SolidFuel" {
                set hasSolid to true.
                set hasProp to true.
                set sf to sf + r:AMOUNT.
            } else if r:NAME = "MonoPropellant" {
                set hasProp to true.
                set mp to mp + r:AMOUNT.
            } else if r:NAME = "XenonGas" {
                set hasProp to true.
                set xn to xn + r:AMOUNT.
            } else if r:NAME = "IntakeAir" {
                set hasAir to true.
            }
        }
    }

    if not hasProp { return true. }
    if hasSolid and sf > 0.1 { return true. }
    if hasLF and hasOx {
        if lf > 0.1 and ox > 0.1 { return true. }
        if lf > 0.1 and hasAir { return true. }
        return false.
    }
    if hasLF { return lf > 0.1. }
    if mp > 0.1 or xn > 0.1 { return true. }
    return false.
}

function shouldStage {
    if SHIP:MAXTHRUST <= 0.01 { return true. }
    if not stageHasUsablePropellant() { return true. }
    return false.
}

// --------------------------------------------------------------------------
// PD-регулятор для плавного управления тягой
// Возвращает List(output, error).
// --------------------------------------------------------------------------
function pidThrottle { parameter targetAccel. parameter currentAccel. parameter kp. parameter kd. parameter prevError. parameter dt.
    local error to targetAccel - currentAccel.
    local derivative to 0.

    if dt > 0.001 {
        set derivative to (error - prevError) / dt.
    }

    local output to kp * error + kd * derivative.
    return LIST(output, error).
}