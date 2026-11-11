@lazyglobal on.

parameter apAlt is 0.
parameter peAlt is -1.
parameter incDeg is 0.
parameter tolM is 5.
parameter tolDeg is 0.05.
parameter maxDV is 10000.

run utils.
run align_plane(true).

set otTolM to tolM.
set otTolDeg to tolDeg.
set otMaxDV to maxDV.
set otMinDv to 0.02.
set otBody to SHIP.

set otApAlt to apAlt.
set otPeAlt to peAlt.
set otIncDeg to incDeg.

set otDesPeRad to 1.
set otDesApRad to 1.

set otOwnNode to 0.
set otOwnTime to 0.
set otHasOwnNode to false.

// ==========================================================================
// Свои ноды: только они удаляются
// ==========================================================================
function otFindOwnNode {
    if not otHasOwnNode or otOwnTime <= 0 {
        return 0.
    }

    for nd in ALLNODES {
        if ABS(nd:TIME - otOwnTime) < 1 {
            return nd.
        }
    }

    return 0.
}

function otClearOwnNode {
    local nd to otFindOwnNode().

    if nd <> 0 {
        remove nd.
    }

    set otOwnNode to 0.
    set otOwnTime to 0.
    set otHasOwnNode to false.
}

function otCalibrateNode { parameter nd.
    local bad to LIST(false, V(0, 0, 0), V(0, 0, 0), V(0, 0, 0)).

    if nd = 0 {
        return bad.
    }

    set nd:RADIALOUT to 0.
    set nd:NORMAL to 0.
    set nd:PROGRADE to 0.
    wait 0.05.

    set nd:NORMAL to 1.
    wait 0.05.

    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 {
        return bad.
    }

    local axN to nd:DELTAV:NORMALIZED.

    set nd:NORMAL to 0.
    set nd:PROGRADE to 1.
    wait 0.05.

    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 {
        return bad.
    }

    local axP to nd:DELTAV:NORMALIZED.

    set nd:PROGRADE to 0.
    set nd:RADIALOUT to 1.
    wait 0.05.

    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 {
        return bad.
    }

    return LIST(true, nd:DELTAV:NORMALIZED, axN, axP).
}

function otAddNode { parameter dvVec. parameter etaSec.
    if not vecIsOk(dvVec) {
        return false.
    }

    local dvMag to dvVec:MAG.

    if not isFiniteVal(dvMag) or dvMag < otMinDv or dvMag > otMaxDV {
        return false.
    }

    otClearOwnNode.

    local cnt to 0.

    for nd in ALLNODES {
        set cnt to cnt + 1.
    }

    if cnt > 0 {
        print "orbit_to: есть чужая нода. Удали её вручную.".
        return false.
    }

    local nd to NODE(TIMESPAN(MAX(5, etaSec)), 0, 1, 0).
    add nd.
    wait 0.3.

    if not HASNODE {
        return false.
    }

    local ax to otCalibrateNode(NEXTNODE).

    if not ax[0] {
        remove NEXTNODE.
        return false.
    }

    set NEXTNODE:RADIALOUT to VDOT(dvVec, ax[1]).
    set NEXTNODE:NORMAL to VDOT(dvVec, ax[2]).
    set NEXTNODE:PROGRADE to VDOT(dvVec, ax[3]).
    wait 0.2.

    local misDeg to VANG(NEXTNODE:DELTAV, dvVec).

    if not isFiniteVal(misDeg) or misDeg > 5 {
        remove NEXTNODE.
        return false.
    }

    set otOwnNode to NEXTNODE.
    set otOwnTime to NEXTNODE:TIME.
    set otHasOwnNode to true.

    return true.
}

function otExecNode {
    if not otHasOwnNode or not HASNODE {
        return false.
    }

    if ABS(NEXTNODE:TIME - otOwnTime) > 1 {
        return false.
    }

    unlock THROTTLE.
    unlock STEERING.

    run exec_node(0, 0.05).

    otClearOwnNode.
    wait 1.

    return true.
}

// ==========================================================================
// Нормаль целевой плоскости для заданного наклонения
// incDeg <= 0 -> не менять
// ==========================================================================
function otTargetNormal { parameter incVal.
    local rVec to bodyPosAt(0).
    local vVec to SHIP:VELOCITY:ORBIT.
    local nCur to orbitNormal(rVec, vVec).

    if not vecIsOk(nCur) or nCur:MAG < 0.000001 {
        return V(0, 1, 0).
    }

    set nCur to nCur:NORMALIZED.

    if incVal <= 0 {
        return nCur.
    }

    // Северный полюс тела. Для стандартных тел KSP это近似 V(0,1,0).
    local nEq to V(0, 1, 0).
    local incCur to VANG(nCur, nEq).

    if not isFiniteVal(incCur) {
        return nCur.
    }

    local dInc to incVal - incCur.

    if ABS(dInc) <= otTolDeg {
        return nCur.
    }

    local nodeLine to VCRS(nEq, nCur).

    if not vecIsOk(nodeLine) or nodeLine:MAG < 0.000001 {
        if vecIsOk(rVec) and rVec:MAG > 1 {
            local rHat to rVec:NORMALIZED.
            set nodeLine to rHat - (VDOT(rHat, nEq) * nEq).
        }
    }

    if not vecIsOk(nodeLine) or nodeLine:MAG < 0.000001 {
        set nodeLine to V(1, 0, 0).
    }

    set nodeLine to nodeLine:NORMALIZED.

    local nP to ANGLEAXIS(dInc, nodeLine) * nCur.
    local nM to ANGLEAXIS((-1) * dInc, nodeLine) * nCur.

    local errP to 999.
    local errM to 999.

    if vecIsOk(nP) and nP:MAG > 0.000001 {
        set errP to ABS(VANG(nP:NORMALIZED, nEq) - incVal).
    }

    if vecIsOk(nM) and nM:MAG > 0.000001 {
        set errM to ABS(VANG(nM:NORMALIZED, nEq) - incVal).
    }

    if errM < errP and vecIsOk(nM) and nM:MAG > 0.000001 {
        return nM:NORMALIZED.
    }

    if vecIsOk(nP) and nP:MAG > 0.000001 {
        return nP:NORMALIZED.
    }

    return nCur.
}

function otApsisEta { parameter wantPe.
    local etaVal to 0.

    if wantPe {
        set etaVal to SHIP:ORBIT:ETA:PERIAPSIS.
    } else {
        set etaVal to SHIP:ORBIT:ETA:APOAPSIS.
    }

    if not isFiniteVal(etaVal) or etaVal < 0 {
        if isFiniteVal(SHIP:ORBIT:ECCENTRICITY) and SHIP:ORBIT:ECCENTRICITY < 0.001 {
            set etaVal to 0.
        } else {
            return -1.
        }
    }

    return MAX(5, etaVal).
}

// ==========================================================================
// Импульсы на апсидах
// ==========================================================================
function otBurnPeToAp { parameter targetAp.
    if SHIP:ORBIT:ECCENTRICITY >= 1 {
        return false.
    }

    local muVal to otBody:MU.
    local aCur to SHIP:ORBIT:SEMIMAJORAXIS.
    local rPe to SHIP:ORBIT:PERIAPSIS.
    local minSep to MAX(10, otTolM).

    if not isFiniteVal(targetAp) or targetAp < rPe - minSep {
        return false.
    }

    if targetAp < rPe {
        set targetAp to rPe.
    }

    local aNew to (rPe + targetAp) / 2.
    local vCurArg to muVal * (2 / rPe - 1 / aCur).
    local vNewArg to muVal * (2 / rPe - 1 / aNew).

    if vCurArg <= 0 or vNewArg <= 0 {
        return false.
    }

    local dvScalar to SQRT(vNewArg) - SQRT(vCurArg).

    if ABS(dvScalar) < otMinDv {
        return true.
    }

    local etaVal to otApsisEta(true).

    if etaVal < 0 {
        return false.
    }

    local velVec to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + etaVal)):ORBIT.

    if not vecIsOk(velVec) or velVec:MAG < 1 {
        return false.
    }

    if not otAddNode(dvScalar * velVec:NORMALIZED, etaVal) {
        return false.
    }

    return otExecNode().
}
function otBurnApToPe { parameter targetPe.
    if SHIP:ORBIT:ECCENTRICITY >= 1 {
        return false.
    }

    local muVal to otBody:MU.
    local aCur to SHIP:ORBIT:SEMIMAJORAXIS.
    local rAp to SHIP:ORBIT:APOAPSIS.
    local minSep to MAX(10, otTolM).

    if not isFiniteVal(targetPe) or targetPe > rAp + minSep {
        return false.
    }

    if targetPe > rAp {
        set targetPe to rAp.
    }

    if targetPe < otBody:RADIUS + 1000 {
        set targetPe to otBody:RADIUS + 1000.
    }

    local aNew to (targetPe + rAp) / 2.
    local vCurArg to muVal * (2 / rAp - 1 / aCur).
    local vNewArg to muVal * (2 / rAp - 1 / aNew).

    if vCurArg <= 0 or vNewArg <= 0 {
        return false.
    }

    local dvScalar to SQRT(vNewArg) - SQRT(vCurArg).

    if ABS(dvScalar) < otMinDv {
        return true.
    }

    local etaVal to otApsisEta(false).

    if etaVal < 0 {
        return false.
    }

    local velVec to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + etaVal)):ORBIT.

    if not vecIsOk(velVec) or velVec:MAG < 1 {
        return false.
    }

    if not otAddNode(dvScalar * velVec:NORMALIZED, etaVal) {
        return false.
    }

    return otExecNode().
}

function otApsidalLoop {
    local iter to 0.
    local stall to 0.
    local lastPe to -1.
    local lastAp to -1.
    local margin to MAX(100, otTolM * 10).
    local easyTol to MAX(otTolM, 25).

    until iter >= 12 {
        if SHIP:ORBIT:ECCENTRICITY >= 1 {
            print "orbit_to: орбита разомкнулась.".
            return false.
        }

        local rPe to otBody:RADIUS + SHIP:ORBIT:PERIAPSIS.
        local rAp to otBody:RADIUS + SHIP:ORBIT:APOAPSIS.

        if not isFiniteVal(rPe) or not isFiniteVal(rAp) {
            return false.
        }

        print "otLoop curPe=" + ROUND(rPe - otBody:RADIUS) +
              " curAp=" + ROUND(rAp - otBody:RADIUS) +
              " dstPe=" + ROUND(otDesPeRad - otBody:RADIUS) +
              " dstAp=" + ROUND(otDesApRad - otBody:RADIUS).

        if ABS(rPe - otDesPeRad) <= otTolM and ABS(rAp - otDesApRad) <= otTolM {
            return true.
        }

        if ABS(rPe - otDesPeRad) <= easyTol and ABS(rAp - otDesApRad) <= easyTol {
            print "orbit_to: мягкий допуск достигнут.".
            return true.
        }

        if ABS(rPe - lastPe) < 1 and ABS(rAp - lastAp) < 1 {
            set stall to stall + 1.
        } else {
            set stall to 0.
        }

        if stall >= 2 {
            print "orbit_to: нет прогресса по Ap/Pe.".
            return false.
        }

        set lastPe to rPe.
        set lastAp to rAp.

        local doneStep to false.

        // Если целевой Ap ниже текущего Pe, сначала гасим Ap в перицентре.
        // Иначе попытка опустить Pe на далёком апоцентре даёт микроскопический dV.
        if otDesApRad < rPe - margin {
            if rAp > rPe + margin * 2 {
                if not otBurnPeToAp(rPe + margin) {
                    return false.
                }

                set doneStep to true.
            } else {
                // Уже почти круговая на текущем Pe. Теперь опускаем Pe чуть ниже цели,
                // чтобы финальная подстройка попала в допуск.
                local tempPe to otDesPeRad - MAX(5, otTolM).

                if tempPe < otBody:RADIUS + 1000 {
                    set tempPe to otBody:RADIUS + 1000.
                }

                if tempPe > otDesApRad - MAX(5, otTolM) {
                    set tempPe to otDesApRad - MAX(5, otTolM).
                }

                if tempPe >= rAp - margin {
                    return false.
                }

                if not otBurnApToPe(tempPe) {
                    return false.
                }

                set doneStep to true.
            }
        }

        // Если целевой Pe выше текущего Ap, сначала поднимаем Ap.
        if not doneStep and otDesPeRad > rAp + margin {
            local tempAp to MAX(otDesApRad, otDesPeRad + margin).

            if tempAp <= rPe + margin {
                return false.
            }

            if not otBurnPeToAp(tempAp) {
                return false.
            }

            set doneStep to true.
        }

        // Обычная ветка: доводим Ap/Pe по относительной ошибке.
        if not doneStep {
            local needAp to ABS(rAp - otDesApRad) > otTolM.
            local needPe to ABS(rPe - otDesPeRad) > otTolM.
            local relAp to ABS(rAp - otDesApRad) / MAX(1, otDesApRad).
            local relPe to ABS(rPe - otDesPeRad) / MAX(1, otDesPeRad).

            if needAp and (not needPe or relAp >= relPe) {
                if not otBurnPeToAp(otDesApRad) {
                    return false.
                }
            } else if needPe {
                if not otBurnApToPe(otDesPeRad) {
                    return false.
                }
            } else if needAp {
                if not otBurnPeToAp(otDesApRad) {
                    return false.
                }
            } else {
                return true.
            }
        }

        set iter to iter + 1.
        wait 1.
    }

    local rPeF to otBody:RADIUS + SHIP:ORBIT:PERIAPSIS.
    local rApF to otBody:RADIUS + SHIP:ORBIT:APOAPSIS.

    return ABS(rPeF - otDesPeRad) <= MAX(otTolM * 5, 50) and
           ABS(rApF - otDesApRad) <= MAX(otTolM * 5, 50).
}

// ==========================================================================
// Main
// ==========================================================================
function otMain {
    print "=== ORBIT_TO ===".

    set WARP to 0.
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.

    otClearOwnNode.

    if HASNODE {
        print "orbit_to: есть чужая маневровая нода. Удали её вручную.".
        return false.
    }

    if SHIP:MAXTHRUST <= 0.01 {
        print "orbit_to: нет тяги.".
        return false.
    }

    if SHIP:ORBIT:ECCENTRICITY >= 1 {
        print "orbit_to: нет замкнутой орбиты.".
        return false.
    }

    // Если ничего не задано — выходим без действий.
    if otApAlt <= 0 and otPeAlt < 0 and otIncDeg <= 0 {
        print "orbit_to: нет целевых параметров.".
        return true.
    }

    // orbit_to(apAlt) -> круговая на apAlt.
    if otPeAlt < 0 and otApAlt > 0 {
        set otPeAlt to otApAlt.
    }

    // orbit_to(0, peAlt) -> круговая на peAlt.
    if otApAlt <= 0 and otPeAlt >= 0 {
        set otApAlt to otPeAlt.
    }

    // Только наклонение: сохраняем текущие Ap/Pe.
    if otApAlt <= 0 and otPeAlt < 0 {
        set otPeAlt to SHIP:ORBIT:PERIAPSIS.
        set otApAlt to SHIP:ORBIT:APOAPSIS.
    }

    if otPeAlt < 1000 {
        set otPeAlt to 1000.
    }

    if otApAlt < otPeAlt {
        local tmpAlt to otApAlt.
        set otApAlt to otPeAlt.
        set otPeAlt to tmpAlt.
    }

    set otBody to SHIP:ORBIT:BODY.
    set otDesPeRad to BODY:RADIUS + otPeAlt.
    set otDesApRad to BODY:RADIUS + otApAlt.
print "otDBG body=" + otBody:NAME + " BODY=" + BODY:NAME.
print "otDBG pe=" + SHIP:ORBIT:PERIAPSIS + " ap=" + SHIP:ORBIT:APOAPSIS + " ecc=" + SHIP:ORBIT:ECCENTRICITY.
print "otDBG sma=" + SHIP:ORBIT:SEMIMAJORAXIS.
print "otDBG etaPe=" + SHIP:ORBIT:ETA:PERIAPSIS + " etaAp=" + SHIP:ORBIT:ETA:APOAPSIS.
print "otDBG wantPe=" + otPeAlt + " wantAp=" + otApAlt.
    print "Цель: Pe=" + ROUND(otPeAlt) +
          " м, Ap=" + ROUND(otApAlt) +
          " м, inc=" + ROUND(otIncDeg, 3) + " deg".

    // Наклонение.
    if otIncDeg > 0 {
        local curInc to SHIP:ORBIT:INCLINATION.

        if isFiniteVal(curInc) and ABS(curInc - otIncDeg) > otTolDeg {
            print "Меняю наклонение: " + ROUND(curInc, 3) +
                  " -> " + ROUND(otIncDeg, 3) + " deg".

            local nTar to otTargetNormal(otIncDeg).

            local alignRes to alignPlaneToNormal(
                nTar,
                -1,
                otTolDeg,
                3,
                true,
                true,
                0.5,
                otMaxDV,
                5,
                50,
                10000,
                30,
                3
            ).

            if not alignRes[0] {
                print "orbit_to: preserve=true не сработал, пробую preserve=false.".

                set alignRes to alignPlaneToNormal(
                    nTar,
                    -1,
                    otTolDeg,
                    3,
                    true,
                    false,
                    0.5,
                    otMaxDV,
                    5,
                    50,
                    10000,
                    30,
                    3
                ).
            }

            if not alignRes[0] {
                print "orbit_to: совмещение наклонения не выполнено.".
                return false.
            }
        }
    }

    // Ap/Pe.
    if not otApsidalLoop() {
        print "orbit_to: не удалось выйти на целевую орбиту.".
        return false.
    }

    print "=== ORBIT_TO ЗАВЕРШЁН ===".
    print "Ap=" + ROUND(SHIP:ORBIT:APOAPSIS) +
          " Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) +
          " inc=" + ROUND(SHIP:ORBIT:INCLINATION, 3) + " deg".

    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.

    return true.
}

otMain().