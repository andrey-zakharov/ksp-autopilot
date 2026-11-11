@lazyglobal on.
parameter debugOn is false.
parameter targetName is "".
parameter destPeAlt is 20000.
parameter destApAlt is 20000.
parameter maxWindowDurationDays is 365.

run utils.
run align_inc(true).
run align_plane(true).

// --- Параметры ---
set tbDebug to debugOn.           // печатать отладочные сообщения
set tbMaxWinDays to maxWindowDurationDays. // максимум дней ожидания окна перехода
set tbTarget to SHIP.             // целевое тело/судно (заполняется в tbMain)
set tbDestPeAlt to destPeAlt.     // целевой перицентр, м над поверхностью
set tbDestApAlt to destApAlt.     // целевой апоцентр, м над поверхностью
set tbSafePeAlt to 20000.         // безопасный перицентр (20 км или атмосфера+2 км)
set tbMaxBurn to 10000.           // лимит dV одной ноды, м/с
set tbMaxMid to 300.              // лимит dV одной mid-course коррекции, м/с
set tbMaxMidBurns to 6.           // максимум коррекций на переходе
set tbPlaneMin to 0.5.            // порог наклонения для совмещения, град
set tbHardInc to 10.              // наклонение выше которого без ноды — отмена
set tbMaxWinOrb to 40.            // максимум витков ожидания окна
set tbPhaseTol to 5.              // допуск фазового угла, град

// --- Ноды (состояние) ---
set tbHasNd1 to false.            // есть ли первая нода
set tbHasNd2 to false.            // есть ли вторая нода (совмещение плоскостей)
set tbNd1 to 0.                   // ссылка на первую ноду
set tbNd2 to 0.                   // ссылка на вторую ноду
set tbNd1Time to 0.               // абсолютное время первой ноды
set tbNd2Time to 0.               // абсолютное время второй ноды

function tbLog { parameter m.
    if tbDebug { print m. }
}

// ==========================================================================
// Ноды: только свои, чужие не трогаем
// ==========================================================================
function tbNodeCount {
    local c to 0.
    for n in ALLNODES { set c to c + 1. }
    return c.
}
function tbOwnCount {
    local c to 0.
    if tbHasNd1 { set c to c + 1. }
    if tbHasNd2 { set c to c + 1. }
    return c.
}
function tbHasForeign {
    return tbNodeCount > tbOwnCount.
}
function tbFindNodeByTime { parameter t.
    for n in ALLNODES {
        if ABS(n:TIME - t) < 1 { return n. }
    }
    return 0.
}
function tbClearNd1 {
    if tbHasNd1 {
        if tbFindNodeByTime(tbNd1Time) <> 0 { remove tbFindNodeByTime(tbNd1Time). }
        set tbHasNd1 to false.
        set tbNd1 to 0.
        set tbNd1Time to 0.
    }
}
function tbClearNd2 {
    if tbHasNd2 {
        if tbFindNodeByTime(tbNd2Time) <> 0 { remove tbFindNodeByTime(tbNd2Time). }
        set tbHasNd2 to false.
        set tbNd2 to 0.
        set tbNd2Time to 0.
    }
}
function tbClearPlan {
    tbClearNd2.
    tbClearNd1.
}

function tbCalibrateNode { parameter nd.
    local bad to LIST(false, V(0, 0, 0), V(0, 0, 0), V(0, 0, 0)).
    if nd = 0 { return bad. }
    set nd:RADIALOUT to 0. set nd:NORMAL to 0. set nd:PROGRADE to 0.
    wait 0.05.
    set nd:NORMAL to 1. wait 0.05.
    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 { return bad. }
    local axN to nd:DELTAV:NORMALIZED.
    set nd:NORMAL to 0. set nd:PROGRADE to 1. wait 0.05.
    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 { return bad. }
    local axP to nd:DELTAV:NORMALIZED.
    set nd:PROGRADE to 0. set nd:RADIALOUT to 1. wait 0.05.
    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 { return bad. }
    return LIST(true, nd:DELTAV:NORMALIZED, axN, axP).
}

function tbAddVectorNode { parameter dvVec. parameter etaSec. parameter isSecond.
    if not vecIsOk(dvVec) { return false. }
    if not isFiniteVal(dvVec:MAG) or dvVec:MAG < 0.5 or dvVec:MAG > tbMaxBurn { return false. }
    if isSecond {
        if not tbHasNd1 or tbNodeCount <> 1 { return false. }
    } else {
        tbClearPlan.
        if tbHasForeign {
            print "transfer: есть чужая нода. Удали её вручную.".
            return false.
        }
    }
    add NODE(TIMESPAN(MAX(5, etaSec)), 0, 1, 0).
    if isSecond {
        set tbNd2 to NEXTNODE. set tbHasNd2 to true. set tbNd2Time to NEXTNODE:TIME.
    } else {
        set tbNd1 to NEXTNODE. set tbHasNd1 to true. set tbNd1Time to NEXTNODE:TIME.
    }
    wait 0.3.
    if tbFindNodeByTime(NEXTNODE:TIME) = 0 {
        if isSecond { set tbHasNd2 to false. } else { set tbHasNd1 to false. }
        return false.
    }
    local ax to tbCalibrateNode(NEXTNODE).
    if not ax[0] {
        remove NEXTNODE.
        if isSecond { set tbHasNd2 to false. } else { set tbHasNd1 to false. }
        return false.
    }
    set NEXTNODE:RADIALOUT to VDOT(dvVec, ax[1]).
    set NEXTNODE:NORMAL to VDOT(dvVec, ax[2]).
    set NEXTNODE:PROGRADE to VDOT(dvVec, ax[3]).
    wait 0.2.
    local lim to 2.
    if isSecond { set lim to 3. }
    if not isFiniteVal(VANG(NEXTNODE:DELTAV, dvVec)) or VANG(NEXTNODE:DELTAV, dvVec) > lim {
        remove NEXTNODE.
        if isSecond { set tbHasNd2 to false. } else { set tbHasNd1 to false. }
        return false.
    }
    return true.
}

function tbExecNext { parameter tVal.
    if not HASNODE or ABS(NEXTNODE:TIME - tVal) > 1 { return false. }
    unlock THROTTLE.
    unlock STEERING.
    run exec_node(0, 0.1).
    wait 0.5.
    if tbFindNodeByTime(tVal) <> 0 {
        remove tbFindNodeByTime(tVal).
        return false.
    }
    return true.
}

function tbExecPlan {
    if not tbHasNd1 { return false. }
    if not tbExecNext(tbNd1Time) {
        tbClearPlan.
        return false.
    }
    set tbHasNd1 to false. set tbNd1 to 0. set tbNd1Time to 0.
    wait 1.
    if tbHasNd2 {
        if tbFindNodeByTime(tbNd2Time) = 0 {
            set tbHasNd2 to false.
            return false.
        }
        if not tbExecNext(tbNd2Time) {
            tbClearNd2.
            return false.
        }
        set tbHasNd2 to false. set tbNd2 to 0. set tbNd2Time to 0.
        wait 1.
    }
    return true.
}

function tbBurnVector { parameter dvVec. parameter etaSec.
    if not vecIsOk(dvVec) or dvVec:MAG < 0.5 { return true. }
    if not tbAddVectorNode(dvVec, etaSec, false) { return false. }
    if not tbExecNext(tbNd1Time) {
        tbClearNd1.
        return false.
    }
    set tbHasNd1 to false. set tbNd1 to 0. set tbNd1Time to 0.
    return true.
}

// ==========================================================================
// Орбитальные хелперы
// ==========================================================================
function tbNormalOf { parameter obj.
    if obj:NAME = SHIP:NAME {
        return orbitNormal(bodyPosAt(0), SHIP:VELOCITY:ORBIT).
    }
    return orbitNormal(
        ORBITAT(obj, TIME:SECONDS):POSITION,
        ORBITAT(obj, TIME:SECONDS):VELOCITY:ORBIT).
}

function tbAcuteNormal { parameter refN. parameter tgtN.
    if not vecIsOk(tgtN) or tgtN:MAG < 0.5 { return refN:NORMALIZED. }
    if VANG(refN:NORMALIZED, tgtN:NORMALIZED) > 90 { return (-1) * tgtN:NORMALIZED. }
    return tgtN:NORMALIZED.
}

// Истинная аномалия -> средняя, через getMA из align_inc
function tbMFromNu { parameter nuDeg. parameter ecc.
    local e to clamp(ecc, 0, 0.99).
    if e < 0.000001 { return normDeg360(nuDeg). }
    return normDeg360(getMA(nuDeg, e)).
}

function tbElemFromState { parameter rVec. parameter vVec. parameter muVal.
    local bad to LIST(false, V(0,0,0), V(0,0,0), V(0,0,0), 0, 0, 0, 0, 0, 0).
    if not vecIsOk(rVec) or not vecIsOk(vVec) { return bad. }
    local hVec to VCRS(rVec, vVec).
    if not vecIsOk(hVec) or hVec:MAG < 0.000001 { return bad. }
    local eVec to VCRS(vVec, hVec) / muVal - rVec:NORMALIZED.
    if not vecIsOk(eVec) or eVec:MAG >= 0.995 { return bad. }
    local eHat to rVec:NORMALIZED.
    if eVec:MAG > 0.000001 { set eHat to eVec:NORMALIZED. }
    local qVec to VCRS(hVec:NORMALIZED, eHat).
    if not vecIsOk(qVec) or qVec:MAG < 0.000001 { return bad. }
    local pSem to hVec:MAG * hVec:MAG / muVal.
    local energy to vVec:MAG * vVec:MAG / 2 - muVal / rVec:MAG.
    if not isFiniteVal(energy) or energy >= 0 { return bad. }
    local sma to pSem / MAX(0.000001, 1 - eVec:MAG * eVec:MAG).
    local per to 2 * CONSTANT:PI * SQRT(sma * sma * sma / muVal).
    if not isFiniteVal(per) or per <= 0 { return bad. }
    local nuNow to normDeg360(ARCTAN2(VDOT(rVec, qVec:NORMALIZED), VDOT(rVec, eHat))).
    return LIST(true, hVec:NORMALIZED, eHat, qVec:NORMALIZED, eVec:MAG, pSem, per, 360 / per, tbMFromNu(nuNow, eVec:MAG), sma).
}

function tbRadiusFromDir { parameter el. parameter dirVec.
    if not el[0] or not vecIsOk(dirVec) { return -1. }
    local den to 1 + el[4] * COS(normDeg360(ARCTAN2(VDOT(dirVec, el[3]), VDOT(dirVec, el[2])))).
    if den < 0.000001 { return -1. }
    return el[5] / den.
}

// ==========================================================================
// Аналитическая фаза перехода
// ==========================================================================
function tbPhaseEta {
    parameter rS. parameter vS. parameter perS. parameter tH.
    parameter minEta. parameter maxEta.
    if not vecIsOk(rS) or not vecIsOk(vS) { return -1. }
    if not isFiniteVal(tH) or tH <= 0 { return -1. }
    local nS to orbitNormal(rS, vS).
    local rT to ORBITAT(tbTarget, TIME:SECONDS):POSITION.
    if not vecIsOk(rT) { return -1. }
    local rTp to rT - (VDOT(rT, nS) * nS).
    if not vecIsOk(rTp) or rTp:MAG < 1 { set rTp to rT. }
    local nTd to 360 / tbTargetPeriod.
    local nT to tbNormalOf(tbTarget).
    if vecIsOk(nT) and VDOT(nT, nS) < 0 { set nTd to -nTd. }
    local omRel to nTd - 360 / MAX(60, perS).
    if ABS(omRel) < 0.000001 { return -1. }
    local etaPha to wrap180Deg(
        wrap180Deg(180 - nTd * tH) - signedAngleDeg(rS, rTp, nS)) / omRel.
    until etaPha >= minEta { set etaPha to etaPha + ABS(360 / omRel). }
    if etaPha > maxEta { return -1. }
    tbLog("Phase: eta=" + ROUND(etaPha, 1) + " omRel=" + ROUND(omRel, 6)).
    return etaPha.
}

function tbDepartureEval { parameter etaSec.
    local bad to LIST(999999, 0, V(0, 0, 0), 0, 0).
    if not isFiniteVal(etaSec) or etaSec < 5 { return bad. }
    local rS to bodyPosAt(etaSec).
    if not vecIsOk(rS) or rS:MAG < BODY:RADIUS + 1000 { return bad. }
    local vS to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + etaSec)):ORBIT.
    if not vecIsOk(vS) or vS:MAG < 1 { return bad. }
    local aTr to (rS:MAG + tbTargetSMA) / 2.
    local vel to tbCentralMu * (2 / rS:MAG - 1 / aTr).
    if not isFiniteVal(vel) or vel <= 0 { return bad. }
    local dv to SQRT(vel) - vS:MAG.
    if ABS(dv) < 0.5 or ABS(dv) > tbMaxBurn { return bad. }
    local tTr to CONSTANT:PI * SQRT(aTr * aTr * aTr / tbCentralMu).
    local rT to ORBITAT(tbTarget, TIME:SECONDS + etaSec + tTr):POSITION.
    if not vecIsOk(rT) or rT:MAG < 1 { return bad. }
    local nS to orbitNormal(rS, vS).
    local tP to rT:NORMALIZED - (VDOT(rT:NORMALIZED, nS) * nS).
    if not vecIsOk(tP) or tP:MAG < 0.001 { set tP to rT:NORMALIZED. }
    local err to VANG((-1) * rS:NORMALIZED, tP:NORMALIZED) +
                 ABS(rT:MAG - tbTargetSMA) / tbTargetSMA * 10.
    if SHIP:ORBIT:ECCENTRICITY > 0.02 {
        set err to err + ABS(VDOT(vS, rS:NORMALIZED)) / 5.
    }
    if not vecIsOk(dv * vS:NORMALIZED) { return bad. }
    return LIST(err, dv, dv * vS:NORMALIZED, etaSec, tTr).
}

function tbRefineEta { parameter minEta. parameter maxEta. parameter guess.
    local bad to LIST(-1, 999999, V(0, 0, 0), 0, 0).
    if not isFiniteVal(guess) or guess < minEta or guess > maxEta { return bad. }
    local span to MAX(60, MAX(60, SHIP:ORBIT:PERIOD) * 0.08).
    local step to MAX(2, MIN(span, (maxEta - minEta)) / 20).
    local lo to MAX(minEta, guess - span).
    local hi to MIN(maxEta, guess + span).
    local best to LIST(999999, 0, V(0, 0, 0), -1, 0).
    local t to lo.
    local cnt to 0.
    until t > hi {
        if tbDepartureEval(t)[0] < best[0] { set best to tbDepartureEval(t). }
        set t to t + step.
        set cnt to cnt + 1.
        if cnt >= 5 { set cnt to 0. wait 0. }
    }
    if best[3] < 0 { return bad. }
    set lo to MAX(minEta, best[3] - step).
    set hi to MIN(maxEta, best[3] + step).
    set step to MAX(0.5, step / 5).
    set t to lo.
    set cnt to 0.
    until t > hi {
        if tbDepartureEval(t)[0] < best[0] { set best to tbDepartureEval(t). }
        set t to t + step.
        set cnt to cnt + 1.
        if cnt >= 3 { set cnt to 0. wait 0. }
    }
    if best[3] < 0 { return bad. }
    return LIST(best[3], best[0], best[2], best[1], best[4]).
}

// ==========================================================================
// Вторая нода: совмещение плоскости в узле переходной орбиты
// ==========================================================================
function tbPlaneNodeOnTransfer {
    parameter rDep. parameter vDep. parameter dv1. parameter eta1.
    local el to tbElemFromState(rDep, vDep + dv1, tbCentralMu).
    if not el[0] { return LIST(false, 0, 0). }
    local nGoal to tbAcuteNormal(el[1], tbTargetNrm).
    local uVec to VCRS(el[1], nGoal).
    if not vecIsOk(uVec) or uVec:MAG < 0.000001 { return LIST(true, 0, 0). }
    set uVec to uVec:NORMALIZED.
    local apoR to (rDep:MAG + tbTargetSMA) / 2 * (1 + el[4]).
    local minR to MAX(BODY:RADIUS + 5000, apoR * 0.7).
    local bestR to -1.
    local bestDt to -1.
    local bestDir to V(0, 0, 0).
    local sIdx to 0.
    until sIdx >= 2 {
        local dirVec to uVec.
        if sIdx = 1 { set dirVec to (-1) * uVec. }
        local rNode to tbRadiusFromDir(el, dirVec).
        if rNode > minR {
            local nuN to normDeg360(ARCTAN2(VDOT(dirVec, el[3]), VDOT(dirVec, el[2]))).
            local mN to tbMFromNu(nuN, el[4]).
            local nuD to normDeg360(ARCTAN2(VDOT(rDep, el[3]), VDOT(rDep, el[2]))).
            local mD to tbMFromNu(nuD, el[4]).
            local dtN to normDeg360(mN - mD) / el[7].
            if dtN < 30 { set dtN to dtN + el[6]. }
            if dtN > 60 and dtN < el[6] * 0.75 and rNode > bestR {
                set bestR to rNode.
                set bestDt to dtN.
                set bestDir to dirVec.
            }
        }
        set sIdx to sIdx + 1.
        wait 0.
    }
    if bestR <= 0 or bestDt <= 0 { return LIST(false, 0, 0). }
    local nu to normDeg360(ARCTAN2(VDOT(bestDir, el[3]), VDOT(bestDir, el[2]))).
    local rN to ((el[2] * COS(nu)) + (el[3] * SIN(nu))):NORMALIZED * bestR.
    local vN to ((el[2] * (-SIN(nu))) + (el[3] * (el[4] + COS(nu)))) * SQRT(tbCentralMu / el[5]).
    if not vecIsOk(rN) or not vecIsOk(vN) { return LIST(false, 0, 0). }
    local rH to rN:NORMALIZED.
    local hN to orbitNormal(rN, vN).
    local nU to tbAcuteNormal(hN, nGoal).
    local nP to nU - (VDOT(nU, rH) * rH).
    if not vecIsOk(nP) or nP:MAG < 0.000001 { return LIST(true, 0, 0). }
    local alpha to VANG(hN, nP:NORMALIZED).
    if alpha < 0.05 { return LIST(true, 0, 0). }
    local bestDv to V(0, 0, 0).
    local bestErr to 999.
    set sIdx to 0.
    until sIdx >= 2 {
        local sg to 1.
        if sIdx = 1 { set sg to -1. }
        local vR to ANGLEAXIS(sg * alpha, rH) * vN.
        local hT to orbitNormal(rN, vR).
        if vecIsOk(hT) and hT:MAG > 0.000001 and VANG(hT, nU) < bestErr {
            set bestErr to VANG(hT, nU).
            set bestDv to vR - vN.
        }
        set sIdx to sIdx + 1.
        wait 0.
    }
    if bestDv:MAG < 0.5 { return LIST(true, 0, 0). }
    if bestDv:MAG > tbMaxBurn { return LIST(false, 0, 0). }
    if not tbAddVectorNode(bestDv, eta1 + bestDt, true) { return LIST(false, 0, 0). }
    return LIST(true, eta1 + bestDt, bestDv:MAG).
}

// ==========================================================================
// Фаза 2: планирование перехода
// ==========================================================================
function tbPlanTransfer {
    tbClearPlan.
    if tbHasForeign {
        print "transfer: есть чужая нода. Удали её вручную.".
        return LIST(false, 0, 0, false).
    }
    local minEta to MAX(30, estimateTurnTime(180, SHIP:MASS) + 10).
    local perShip to MAX(60, SHIP:ORBIT:PERIOD).
    local maxEta to MIN(minEta + perShip * clamp(tbMaxWinOrb, 1, 40), tbMaxWinDays * 86400).
    local elS to tbElemFromState(bodyPosAt(0), SHIP:VELOCITY:ORBIT, tbCentralMu).
    if not elS[0] { return LIST(false, 0, 0, false). }
    local elT to tbElemFromState(ORBITAT(tbTarget, TIME:SECONDS):POSITION, ORBITAT(tbTarget, TIME:SECONDS):VELOCITY:ORBIT, tbCentralMu).
    if not elT[0] { return LIST(false, 0, 0, false). }
    set tbTargetNrm to tbAcuteNormal(elS[1], elT[1]).
    set tbRelInc to VANG(elS[1], tbTargetNrm).
    local outward to tbTargetSMA >= elS[9].
    local depR to BODY:RADIUS + SHIP:ORBIT:PERIAPSIS.
    if not outward { set depR to BODY:RADIUS + SHIP:ORBIT:APOAPSIS. }
    local transA to (depR + tbTargetSMA - TARGET:SOIRADIUS / 2) / 2.
    local tH to CONSTANT:PI * SQRT(transA * transA * transA / tbCentralMu).
    local eta1 to tbPhaseEta(bodyPosAt(0), SHIP:VELOCITY:ORBIT, perShip, tH, minEta, maxEta).
    if eta1 < 0 { return LIST(false, 0, 0, false). }
    local ref to tbRefineEta(minEta, maxEta, eta1).
    if ref[0] >= 0 { set eta1 to ref[0]. }
    local rDep to bodyPosAt(eta1).
    local vDep to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + eta1)):ORBIT.
    if not vecIsOk(rDep) or not vecIsOk(vDep) { return LIST(false, 0, 0, false). }
    print "dep height=" + disMet(rDep:MAG).
    print "target SOI=" + disMet(TARGET:SOIRADIUS).
    print "target sma=" + disMet(tbTargetSMA).
    local aTr to (rDep:MAG + tbTargetSMA - TARGET:SOIRADIUS / 2) / 2.
    local vel to tbCentralMu * (2 / rDep:MAG - 1 / aTr).
    if not isFiniteVal(vel) or vel <= 0 { return LIST(false, 0, 0, false). }
    local dv to SQRT(vel) - vDep:MAG.
    if ABS(dv) < 0.5 or ABS(dv) > tbMaxBurn { return LIST(false, 0, 0, false). }
    if not tbAddVectorNode(dv * vDep:NORMALIZED, eta1, false) { return LIST(false, 0, 0, false). }
    if outward and tbRelInc > tbPlaneMin {
        if not tbPlaneNodeOnTransfer(rDep, vDep, tbNd1:DELTAV, eta1)[0] {
            if tbRelInc > tbHardInc {
                tbClearPlan.
                print "Не удалось построить ноду совмещения при relInc=" + ROUND(tbRelInc, 2) + ".".
                return LIST(false, 0, 0, false).
            }
            print "WARNING: plane node not added, relInc=" + ROUND(tbRelInc, 2) + " deg.".
        }
    }
    print "PLAN eta1=" + ROUND(eta1, 1) + " dv1=" + ROUND(tbNd1:DELTAV:MAG, 1) +
          " tH=" + ROUND(CONSTANT:PI * SQRT(aTr * aTr * aTr / tbCentralMu)) +
          " relInc=" + ROUND(tbRelInc, 2).
    if tbHasNd2 {
        print "PLAN eta2=" + ROUND(tbNd2:ETA, 1) + " dv2=" + ROUND(tbNd2:DELTAV:MAG, 1).
    }
    return LIST(true, eta1, CONSTANT:PI * SQRT(aTr * aTr * aTr / tbCentralMu), tbHasNd2).
}

// ==========================================================================
// Фаза 1: выход из SOI
// ==========================================================================
function tbPhase1 {
    local orig to SHIP:ORBIT:BODY.
    local origSMA to orig:ORBIT:SEMIMAJORAXIS.
    local rO to ORBITAT(orig, TIME:SECONDS):POSITION.
    local vO to ORBITAT(orig, TIME:SECONDS):VELOCITY:ORBIT.
    if not vecIsOk(rO) or not vecIsOk(vO) {
        print "Фаза1: не удалось получить орбиту исходного тела.".
        return false.
    }
    local transA to (origSMA + tbTargetSMA) / 2.
    local tH to CONSTANT:PI * SQRT(transA * transA * transA / tbCentralMu).
    local etaPha to tbPhaseEta(rO, vO, orig:ORBIT:PERIOD, tH, 600, tbMaxWinDays * 86400).
    if etaPha < 0 { print "Фаза1: окно перехода не найдено.". return false. }
    local vInf to ABS(SQRT(tbCentralMu * (2 / origSMA - 1 / transA)) - vO:MAG).
    local dirVec to vO:NORMALIZED.
    if tbTargetSMA < origSMA { set dirVec to (-1) * dirVec. }
    print "Фаза1: окно T+" + ROUND(etaPha) + " с, vInf=" + ROUND(vInf, 1) + " м/с".
    local bestEta to -1.
    local bestAng to 999.
    local step to MAX(5, SHIP:ORBIT:PERIOD / 60).
    local t to 30.
    until t > SHIP:ORBIT:PERIOD {
        local vF to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + t)):ORBIT.
        if vecIsOk(vF) and VANG(vF, dirVec) < bestAng {
            set bestAng to VANG(vF, dirVec).
            set bestEta to t.
        }
        set t to t + step.
        wait 0.
    }
    if bestEta < 0 or bestAng > 5 {
        print "Фаза1: не найден момент вылета (ang=" + ROUND(bestAng, 2) + ").".
        return false.
    }
    local rPark to bodyPosAt(bestEta):MAG.
    local vPark to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + bestEta)):ORBIT:MAG.
    local dv to SQRT(vInf * vInf + 2 * orig:MU / rPark) - vPark.
    if dv < 0.5 or dv > tbMaxBurn {
        print "Фаза1: dV выхода вне лимита: " + ROUND(dv, 1) + ".".
        return false.
    }
    print "Фаза1: выход из SOI, eta=" + ROUND(bestEta, 1) + " dv=" + ROUND(dv, 1).
    if not tbBurnVector(VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + bestEta)):ORBIT:NORMALIZED * dv, bestEta) {
        return false.
    }
    local deadline to TIME:SECONDS + SHIP:ORBIT:PERIOD * 3 + 3600.
    until SHIP:ORBIT:BODY:NAME = tbCentralName {
        if TIME:SECONDS > deadline { print "Фаза1: таймаут выхода из SOI.". return false. }
        kuniverse:timewarp:warpto(TIME:SECONDS + 120).
        wait 1.
    }
    set WARP to 0.
    print "Фаза1: выход на орбиту " + tbCentralName + " выполнен.".
    return true.
}

// ==========================================================================
// Относительное состояние, коррекции, пролёт
// ==========================================================================
function tbRelativeState {
    if SHIP:ORBIT:BODY:NAME = tbTargetName {
        return LIST(bodyPosAt(0), SHIP:VELOCITY:ORBIT).
    }
    return LIST(
        bodyPosAt(0) - ORBITAT(tbTarget, TIME:SECONDS):POSITION,
        SHIP:VELOCITY:ORBIT - ORBITAT(tbTarget, TIME:SECONDS):VELOCITY:ORBIT).
}

function tbHyperPeriapsisRadius { parameter relPos. parameter relVel.
    if not vecIsOk(relPos) or not vecIsOk(relVel) { return -1. }
    if relPos:MAG < 1 or relVel:MAG < 0.01 or tbTargetMu <= 0 { return -1. }
    local h to VCRS(relPos, relVel).
    if not vecIsOk(h) or h:MAG < 1 { return -1. }
    local p to h:MAG * h:MAG / tbTargetMu.
    local en to relVel:MAG * relVel:MAG / 2 - tbTargetMu / relPos:MAG.
    local eccArg to 1 + 2 * en * p / tbTargetMu.
    if eccArg < 0 { set eccArg to 0. }
    return p / (1 + SQRT(eccArg)).
}

function tbTimeToCpa { parameter relPos. parameter relVel.
    if relVel:MAG * relVel:MAG < 0.000001 { return -1. }
    return -VDOT(relPos, relVel) / (relVel:MAG * relVel:MAG).
}

function tbRaisePeriapsisDv {
    parameter relPos. parameter relVel. parameter wantRad. parameter lim.
    local total to V(0, 0, 0).
    local curV to relVel.
    local i to 0.
    until i >= 5 {
        if tbHyperPeriapsisRadius(relPos, curV) <= 0 or
           tbHyperPeriapsisRadius(relPos, curV) >= wantRad - 50 { return total. }
        local h to VCRS(relPos, curV).
        if not vecIsOk(h) or h:MAG < 1 { return total. }
        local p to h:MAG * h:MAG / tbTargetMu.
        local en to curV:MAG * curV:MAG / 2 - tbTargetMu / relPos:MAG.
        local pW to 2 * wantRad + 2 * en * wantRad * wantRad / tbTargetMu.
        if pW <= 0 { return total. }
        local dh to SQRT(tbTargetMu * pW) - h:MAG.
        if dh <= 1 or lim - total:MAG <= 0.5 { return total. }
        local addVec to VCRS(h:NORMALIZED, relPos:NORMALIZED):NORMALIZED * MIN(dh / relPos:MAG, lim - total:MAG).
        set total to total + addVec.
        set curV to curV + addVec.
        set i to i + 1.
    }
    return total.
}

function tbMidcourseCorrect {
    local burns to 0.
    until burns >= tbMaxMidBurns {
        local rel to tbRelativeState().
        local rp to tbHyperPeriapsisRadius(rel[0], rel[1]).
        local tca to tbTimeToCpa(rel[0], rel[1]).
        if rp <= 0 or rp >= tbTargetRad + tbSafePeAlt - 50 or tca < 30 { return burns. }
        local dv to tbRaisePeriapsisDv(rel[0], rel[1], tbTargetRad + tbSafePeAlt, tbMaxMid).
        if dv:MAG < 0.5 { return burns. }
        local acc to MAX(0.1, SHIP:MAXTHRUST / MAX(0.001, SHIP:MASS)).
        local etaCor to clamp(tca * 0.25, 15, tca - dv:MAG / acc / 2 - 20).
        if etaCor < 15 { return burns. }
        print "tbMid: rp=" + ROUND(rp - tbTargetRad) + " dv=" + ROUND(dv:MAG, 1) + " eta=" + ROUND(etaCor, 1).
        if not tbBurnVector(dv, etaCor) { return burns. }
        set burns to burns + 1.
        wait 1.
    }
    return burns.
}

function tbFutureDist { parameter etaSec.
    if SHIP:ORBIT:BODY:NAME = tbTargetName {
        if not vecIsOk(bodyPosAt(etaSec)) { return 1000000000000. }
        return bodyPosAt(etaSec):MAG.
    }
    local d to (bodyPosAt(etaSec) - ORBITAT(tbTarget, TIME:SECONDS + etaSec):POSITION):MAG.
    if not isFiniteVal(d) { return 1000000000000. }
    return d.
}

function tbFindPeriapsisEta { parameter maxScan.
    local lim to MAX(600, maxScan).
    local step to MAX(10, lim / 60).
    local bestT to 0.
    local bestD to tbFutureDist(0).
    local t to step.
    local cnt to 0.
    until t > lim {
        if tbFutureDist(t) < bestD { set bestD to tbFutureDist(t). set bestT to t. }
        set t to t + step.
        set cnt to cnt + 1.
        if cnt >= 5 { set cnt to 0. wait 0. }
    }
    local lo to MAX(0, bestT - step).
    local hi to bestT + step.
    local i to 0.
    set cnt to 0.
    until i >= 20 or hi - lo < 0.5 {
        if tbFutureDist(lo + (hi - lo) / 3) < tbFutureDist(hi - (hi - lo) / 3) {
            set hi to hi - (hi - lo) / 3.
        } else {
            set lo to lo + (hi - lo) / 3.
        }
        set i to i + 1.
        set cnt to cnt + 1.
        if cnt >= 5 { set cnt to 0. wait 0. }
    }
    return (lo + hi) / 2.
}

// ==========================================================================
// Фаза 3: переходная орбита — узел через align_inc
// ==========================================================================
function tbPhase3 {
    print "Фаза3: переходная орбита к " + tbTargetName + ".".

    if VANG(tbNormalOf(SHIP), tbTargetNrm) > tbPlaneMin {
        print "Фаза3: совмещение плоскостей, relInc=" +
              ROUND(VANG(tbNormalOf(SHIP), tbTargetNrm), 2) + " deg.".

        // Аналитический расчёт ближайшего узла через align_inc
        local phiInfo to getNearestPhiToNormal(bodyPosAt(0), SHIP:VELOCITY:ORBIT, tbTargetNrm).
        local etaPlane to getTimeToPhi(phiInfo[0], SHIP:ORBIT:ECCENTRICITY,
                                       MAX(60, SHIP:ORBIT:PERIOD), SHIP:ORBIT:TRUEANOMALY).
        if not isFiniteVal(etaPlane) or etaPlane < 30 { set etaPlane to -1. }

        tbLog("Фаза3: узел через " + ROUND(etaPlane, 1) + " с (phi=" + ROUND(phiInfo[0], 2) + "°)").

        if not alignPlaneToNormal(
            tbTargetNrm, etaPlane, 0.05, 3, true, true,
            0.5, tbMaxBurn, 3, 50, 1000, 30, 3)[0] {
            print "Фаза3: совмещение плоскостей не выполнено.".
            return false.
        }
    }

    tbMidcourseCorrect().

    local deadline to TIME:SECONDS + tbMaxWinDays * 86400.
    until SHIP:ORBIT:BODY:NAME = tbTargetName {
        if SHIP:STATUS = "LANDED" or SHIP:STATUS = "SPLASHED" { return false. }
        if TIME:SECONDS > deadline { print "Фаза3: таймаут SOI.". return false. }
        if tbFutureDist(0) > tbTargetSMA { set WARP to 4. }
        else if tbFutureDist(0) > tbTargetSMA * 0.1 { set WARP to 2. }
        else { set WARP to 0. }
        wait 1.
    }
    set WARP to 0.
    print "Фаза3: вход в SOI " + tbTargetName + ".".
    tbMidcourseCorrect().
    return true.
}

// ==========================================================================
// Фаза 4: захват и округление
// ==========================================================================
function tbPhase4 {
    print "Фаза4: захват.".
    local periEta to tbFindPeriapsisEta(MAX(21600, tbTargetPeriod)).
    local periRad to tbFutureDist(periEta).
    if not isFiniteVal(periRad) or periRad <= tbTargetRad + 100 {
        print "Фаза4: перицентр ниже поверхности.".
        return false.
    }
    local periVel to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + periEta)):ORBIT:MAG.
    if not isFiniteVal(periVel) or periVel < 1 {
        print "Фаза4: некорректная скорость в перицентре.".
        return false.
    }
    local aNew to (periRad + tbTargetRad + tbDestApAlt) / 2.
    local vReq to SQRT(MAX(0, tbTargetMu * (2 / periRad - 1 / aNew))).
    local capDv to vReq - periVel.
    print "Фаза4: eta=" + ROUND(periEta, 1) + " rpAlt=" + ROUND(periRad - tbTargetRad) + " dV=" + ROUND(capDv, 1).
    if ABS(capDv) > 0.5 {
        if ABS(capDv) > tbMaxBurn { print "Фаза4: dV захвата велик.". return false. }
        if not tbAddVectorNode(VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + periEta)):ORBIT:NORMALIZED * capDv, periEta, false) {
            print "Фаза4: нода захвата не создана.".
            return false.
        }
        if not tbExecNext(tbNd1Time) {
            tbClearNd1.
            print "Фаза4: захват не выполнен.".
            return false.
        }
        set tbHasNd1 to false. set tbNd1 to 0. set tbNd1Time to 0.
        print "Фаза4: захват выполнен.".
    }
    if SHIP:ORBIT:BODY:NAME <> tbTargetName { print "Фаза4: корабль не на орбите цели.". return false. }
    if SHIP:ORBIT:ECCENTRICITY >= 1 { print "Фаза4: орбита не замкнута.". return false. }
    run orbit_to(tbDestApAlt, tbDestPeAlt).
    print "=== ПЕРЕХОД ЗАВЕРШЁН ===".
    print "Тело: " + SHIP:ORBIT:BODY:NAME +
          " Ap=" + ROUND(SHIP:ORBIT:APOAPSIS) +
          " Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) +
          " inc=" + ROUND(SHIP:ORBIT:INCLINATION, 3) + " deg".
    return true.
}

// ==========================================================================
// Определение фазы
// ==========================================================================
function tbDetectPhase {
    if SHIP:ORBIT:BODY:NAME = tbTargetName {
        if SHIP:ORBIT:ECCENTRICITY >= 1 { return 4. }
        return 5.
    }
    if SHIP:ORBIT:BODY:NAME = tbCentralName {
        if SHIP:ORBIT:ECCENTRICITY >= 1 or BODY:RADIUS + SHIP:ORBIT:APOAPSIS >= tbTargetSMA * 0.95 { return 3. }
        return 2.
    }
    return 1.
}

// ==========================================================================
// Главная функция
// ==========================================================================
function tbMain {
    print "=== TRANSFER TO BODY ===".
    set WARP to 0.
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    tbClearPlan.
    if tbHasForeign {
        print "Есть чужая маневровая нода. Удали её вручную.".
        return false.
    }
    if targetName <> "" {
        set tbTarget to SHIP.
        for v in ALLVESSELS {
            if v:NAME = targetName { set tbTarget to v. }
        }
        if TARGET:NAME = SHIP:NAME {
            print "Цель '" + targetName + "' не найдена среди судов.".
            return false.
        }
    } else {
        if not HASTARGET { print "Нет цели (TARGET).". return false. }
        set tbTarget to TARGET.
    }
    set tbTgtMu to TARGET:MU.
    set tbTgtRad to TARGET:RADIUS.
    set tbTgtSMA to TARGET:ORBIT:SEMIMAJORAXIS.
    set tbTgtPer to MAX(60, TARGET:ORBIT:PERIOD).
    set tbCenMu to TARGET:ORBIT:BODY:MU.
    set tbCenName to TARGET:ORBIT:BODY:NAME.
    set tbTgtName to TARGET:NAME.
    set tbTargetName to TARGET:NAME.
    set tbCentral to TARGET:ORBIT:BODY.
    set tbCentralName to tbCentral:NAME.
    set tbCentralMu to tbCentral:MU.
    set tbTargetMu to TARGET:MU.
    set tbTargetRad to TARGET:RADIUS.
    set tbTargetSMA to TARGET:ORBIT:SEMIMAJORAXIS.
    set tbTargetPeriod to MAX(60, TARGET:ORBIT:PERIOD).
    if TARGET:ATM:EXISTS {
        set tbSafePeAlt to MAX(tbSafePeAlt, TARGET:ATM:HEIGHT + 5000).
    }
    if tbDestApAlt < tbDestPeAlt {
        set tbDestApAlt to tbDestApAlt + tbDestPeAlt.
        set tbDestPeAlt to tbDestApAlt - tbDestPeAlt.
        set tbDestApAlt to tbDestApAlt - tbDestPeAlt.
    }
    set tbDestPeAlt to MAX(tbDestPeAlt, tbSafePeAlt).
    set tbDestApAlt to MAX(tbDestApAlt, tbDestPeAlt).
    if SHIP:ORBIT:BODY:NAME <> tbCentralName and
       SHIP:ORBIT:BODY:NAME <> tbTargetName and
       SHIP:ORBIT:BODY:ORBIT:BODY:NAME <> tbCentralName {
        print "Исходное тело не связано с центральным телом цели.".
        return false.
    }
    set tbTargetNrm to tbAcuteNormal(tbNormalOf(SHIP), tbNormalOf(tbTarget)).
    local phase to tbDetectPhase().
    print "Фаза: " + phase + " (тело=" + SHIP:ORBIT:BODY:NAME +
          ", цель=" + tbTargetName + ", центр=" + tbCentralName + ")".
    if phase = 1 {
        if not tbPhase1() { return false. }
        set phase to tbDetectPhase().
        print "Фаза после выхода: " + phase + ".".
    }
    if phase = 2 {
        local plan to tbPlanTransfer().
        if not plan[0] { print "Фаза2: план перехода не построен.". return false. }
        if not tbExecPlan() {
            tbClearPlan.
            print "Фаза2: манёвры не выполнены.".
            return false.
        }
        print "Фаза2: манёвры перехода выполнены.".
        set phase to 3.
    }
    if phase = 3 {
        if not tbPhase3() { return false. }
    }
    if phase = 5 {
        run orbit_to(tbDestApAlt, tbDestPeAlt).
        print "=== ПЕРЕХОД ЗАВЕРШЁН ===".
        return true.
    }
    if SHIP:ORBIT:BODY:NAME <> tbTargetName {
        print "Корабль не в SOI цели.".
        return false.
    }
    if not tbPhase4() { return false. }
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    return SHIP:ORBIT:BODY:NAME = tbTargetName and SHIP:ORBIT:ECCENTRICITY < 1.
}

tbMain().