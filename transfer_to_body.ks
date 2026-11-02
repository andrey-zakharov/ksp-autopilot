@lazyglobal on.

parameter debugOn is false.
parameter targetName is "".
parameter destPeAlt is 20000.
parameter destApAlt is 20000.
parameter maxWindowDurationDays is 365.

run utils.
run align_plane(true).

// ==========================================================================
// Global state
// ==========================================================================
set tbDebug to debugOn.
set tbMaxWinDays to maxWindowDurationDays.

set tbTarget to SHIP.
set tbTargetName to "".
set tbCentralName to "".
set tbCentralMu to 1.
set tbTargetMu to 1.
set tbTargetRad to 1.
set tbTargetSMA to 1.
set tbTargetPeriod to 3600.

set tbDestPeAlt to destPeAlt.
set tbDestApAlt to destApAlt.
set tbSafePeAlt to 20000.

set tbMaxBurn to 10000.
set tbMaxMid to 300.
set tbMaxMidBurns to 6.

set tbOwnNode to 0.
set tbHasOwnNode to false.

function tbLog { parameter msg. if tbDebug { print msg. } }

// ==========================================================================
// Node management - only remove our own nodes
// ==========================================================================
function tbClearOwnNode {
    if tbHasOwnNode and tbOwnNode <> 0 {
        remove tbOwnNode.
        set tbHasOwnNode to false.
        set tbOwnNode to 0.
    }
}

function tbCalibrateNode { parameter nd.
    local bad to LIST(false, V(0, 0, 0), V(0, 0, 0), V(0, 0, 0)).
    if nd = 0 { return bad. }
    
    set nd:RADIALOUT to 0.
    set nd:NORMAL to 0.
    set nd:PROGRADE to 0.
    wait 0.05.
    
    set nd:NORMAL to 1.
    wait 0.05.
    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 { return bad. }
    local axN to nd:DELTAV:NORMALIZED.
    
    set nd:NORMAL to 0.
    set nd:PROGRADE to 1.
    wait 0.05.
    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 { return bad. }
    local axP to nd:DELTAV:NORMALIZED.
    
    set nd:PROGRADE to 0.
    set nd:RADIALOUT to 1.
    wait 0.05.
    if not vecIsOk(nd:DELTAV) or nd:DELTAV:MAG < 0.000001 { return bad. }
    local axR to nd:DELTAV:NORMALIZED.
    
    return LIST(true, axR, axN, axP).
}

function tbAddVectorNode { parameter dvVec. parameter etaSec.
    if not vecIsOk(dvVec) { return false. }
    local dvMag to dvVec:MAG.
    if not isFiniteVal(dvMag) or dvMag < 0.5 or dvMag > tbMaxBurn { return false. }
    
    tbClearOwnNode.
    
    local etaVal to MAX(5, etaSec).
    local nd to NODE(TIMESPAN(etaVal), 0, 1, 0).
    add nd.
    wait 0.3.
    
    if not HASNODE { return false. }
    
    local ax to tbCalibrateNode(NEXTNODE).
    if not ax[0] {
        remove NEXTNODE.
        return false.
    }
    
    set NEXTNODE:RADIALOUT to VDOT(dvVec, ax[1]).
    set NEXTNODE:NORMAL to VDOT(dvVec, ax[2]).
    set NEXTNODE:PROGRADE to VDOT(dvVec, ax[3]).
    wait 0.2.
    
    if not isFiniteVal(VANG(NEXTNODE:DELTAV, dvVec)) or VANG(NEXTNODE:DELTAV, dvVec) > 5 {
        remove NEXTNODE.
        return false.
    }
    
    set tbOwnNode to NEXTNODE.
    set tbHasOwnNode to true.
    return true.
}

function tbExecNode {
    if not tbHasOwnNode or not HASNODE { return false. }
    run exec_node.
    tbClearOwnNode.
    wait 1.
    return true.
}

// ==========================================================================
// Safe periapsis calculation
// ==========================================================================
function tbGetSafePeAlt { parameter tarBody.
    if tarBody:ATM:EXISTS {
        return tarBody:ATM:HEIGHT + 2000.
    }
    return 20000.
}

// ==========================================================================
// Проверка: есть ли среди орбитальных патчей патч вокруг указанного тела.
// SHIP:PATCHES возвращает список конических патчей, которые KSP
// рассчитывает с учётом всех текущих маневровых нод.
// ==========================================================================
function tbHasPatchFor { parameter bodyName.
    for patch in SHIP:PATCHES {
        if patch:BODY:NAME = bodyName {
            return true.
        }
    }
    return false.
}

// ==========================================================================
// Определение фазы по текущему состоянию и орбитальным патчам
// ==========================================================================
function tbDetectPhase {
    // Уже в SOI цели
    if SHIP:ORBIT:BODY:NAME = tbTargetName {
        if SHIP:ORBIT:ECCENTRICITY >= 1 { return 4. }
        return 5.
    }

    // На орбите центрального тела
    if SHIP:ORBIT:BODY:NAME = tbCentralName {
        // KSP уже видит переход к цели среди патчей — основной манёвр выполнен
        if tbHasPatchFor(tbTargetName) { return 3. }

        // Гиперболическая орбита — тоже переходная (убегание к цели)
        if SHIP:ORBIT:ECCENTRICITY >= 1 { return 3. }

        return 2.
    }

    // На орбите какого-то другого тела (спутник центрального) — фаза выхода
    return 1.
}

// ==========================================================================
// Phase 1: Escape from origin body
// ==========================================================================
function tbPhase1 {
    print "=== ФАЗА 1: Побег с орбиты исходного тела ===".
    
    local origin to SHIP:ORBIT:BODY.
    local originSMA to origin:ORBIT:SEMIMAJORAXIS.
    local originPeriod to origin:ORBIT:PERIOD.
    
    // Calculate Hohmann transfer from origin to target around central body
    local transA to (originSMA + tbTargetSMA) / 2.
    local tTrans to CONSTANT:PI * SQRT(transA * transA * transA / tbCentralMu).
    
    // Calculate phase angle
    local alpha to (360 / tbTargetPeriod) * tTrans.
    local phiOpt to 180 - alpha.
    
    // Calculate ejection v_inf
    local vInf to ABS(SQRT(tbCentralMu * (2 / originSMA - 1 / transA)) - 
                      SQRT(tbCentralMu / originSMA)).
    
    print "tTrans=" + ROUND(tTrans) + " с, vInf=" + ROUND(vInf, 1) + " м/с".
    
    // Find ejection point - burn when velocity aligns with ejection direction
    local dirVec to origin:ORBIT:VELOCITY:ORBIT:NORMALIZED.
    if tbTargetSMA < originSMA { set dirVec to (-1) * dirVec. }
    
    // Calculate required dV to escape with v_inf
    local rPark to BODY:RADIUS + SHIP:ORBIT:PERIAPSIS.
    local vPark to SHIP:ORBIT:VELOCITY:ORBIT:MAG.
    local dv to SQRT(vInf * vInf + 2 * origin:MU / rPark) - vPark.
    
    if dv < 0.5 or dv > tbMaxBurn {
        print "dV выхода вне лимита: " + ROUND(dv, 1) + ".".
        return false.
    }
    
    // Find best ejection time within one orbit
    local bestEta to -1.
    local bestAng to 999.
    local step to MAX(5, SHIP:ORBIT:PERIOD / 60).
    local t to 30.
    
    until t > SHIP:ORBIT:PERIOD {
        local vF to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + t)):ORBIT.
        if vecIsOk(vF) {
            local ang to VANG(vF, dirVec).
            if ang < bestAng {
                set bestAng to ang.
                set bestEta to t.
            }
        }
        set t to t + step.
        wait 0.
    }
    
    if bestEta < 0 or bestAng > 10 {
        print "Не найден момент вылета.".
        return false.
    }
    
    print "Побег: eta=" + ROUND(bestEta, 1) + " dv=" + ROUND(dv, 1).
    
    local burnVec to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + bestEta)):ORBIT:NORMALIZED * dv.
    if not tbAddVectorNode(burnVec, bestEta) { return false. }
    if not tbExecNode() { return false. }
    
    // Wait for SOI change
    local deadline to TIME:SECONDS + SHIP:ORBIT:PERIOD * 3 + 3600.
    until SHIP:ORBIT:BODY:NAME = tbCentralName {
        if TIME:SECONDS > deadline {
            print "Таймаут выхода из SOI.".
            return false.
        }
        kuniverse:timewarp:warpto(TIME:SECONDS + 120).
        wait 1.
    }
    
    set WARP to 0.
    print "Фаза 1 завершена: выход на орбиту " + tbCentralName + ".".
    return true.
}

// ==========================================================================
// Phase 2: Transfer around central body with SOI targeting
// ==========================================================================
function tbPhase2 {
    print "=== ФАЗА 2: Переход вокруг центрального тела ===".
    
    // Calculate Hohmann transfer
    local r1 to SHIP:ORBIT:SEMIMAJORAXIS.
    local r2 to tbTargetSMA.
    local transA to (r1 + r2) / 2.
    local tTrans to CONSTANT:PI * SQRT(transA * transA * transA / tbCentralMu).
    
    // Phase angle calculation
    local omegaShip to 360 / SHIP:ORBIT:PERIOD.
    local omegaTarget to 360 / tbTargetPeriod.
    local omegaRel to omegaShip - omegaTarget.
    
    if ABS(omegaRel) < 0.000001 {
        print "Синхронные орбиты.".
        return false.
    }
    
    local alpha to omegaTarget * tTrans.
    local phiOpt to 180 - alpha.
    
    // Current phase angle
    local rShip to bodyPosAt(0).
    local rTarget to ORBITAT(tbTarget, TIME:SECONDS):POSITION.

    local hShip to orbitNormal(rShip, SHIP:VELOCITY:ORBIT).
    local crossVec to VCRS(rShip, rTarget).
    local sign to 1.
    if VDOT(crossVec, hShip) < 0 { set sign to -1. }
    local phi0 to sign * VANG(rShip, rTarget).
    local dPhi to phi0 - phiOpt.

    local dtWindow to dPhi / omegaRel.
    
    local synodicPeriod to ABS(360 / omegaRel).
    until dtWindow >= 30 {
        set dtWindow to dtWindow + synodicPeriod.
    }
    
    if dtWindow > tbMaxWinDays * 86400 {
        print "Окно слишком далеко.".
        return false.
    }
    
    print "Окно: dtWindow=" + ROUND(dtWindow) + " с".
    
    // Calculate base prograde burn
    local rDep to bodyPosAt(dtWindow).
    local vDep to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + dtWindow)):ORBIT.
    
    local rMag to rDep:MAG.
    local vMag to vDep:MAG.
    local aTr to (rMag + tbTargetSMA) / 2.
    local velTerm to tbCentralMu * (2 / rMag - 1 / aTr).
    
    if velTerm <= 0 { return false. }
    
    local vTrans to SQRT(velTerm).
    local dvBase to vTrans - vMag.
    
    if ABS(dvBase) < 0.5 or ABS(dvBase) > tbMaxBurn { return false. }
    
    // Initial burn vector - pure prograde
    local burnVec to dvBase * vDep:NORMALIZED.
    
    // Now we need to correct this to hit the target's SOI
    // Get target position at arrival time
    local arrTime to dtWindow + tTrans.
    local targetArrPos to ORBITAT(tbTarget, TIME:SECONDS + arrTime):POSITION.
    
    // Where will we be with the base burn?
    // Create temporary node to check orbit
    if not tbAddVectorNode(burnVec, dtWindow) { return false. }
    
    // Get predicted position at arrival
    local predShipPos to ORBITAT(SHIP, TIME:SECONDS + arrTime):POSITION.
    
    // Calculate miss vector
    local missVec to targetArrPos - predShipPos.
    local missDist to missVec:MAG.
    
    tbLog("Initial miss distance: " + ROUND(missDist) + " м").
    
    // If miss is too large, we need to correct
    if missDist > tbTargetSMA * 0.1 {
        // Calculate correction
        // We need to adjust the burn to reduce the miss
        local rHat to rDep:NORMALIZED.
        local vHat to vDep:NORMALIZED.
        local hVec to VCRS(rDep, vDep).
        local hHat to hVec:NORMALIZED.
        
        // Radial direction (outward from central body)
        local radDir to rHat.
        
        // Normal direction (out of plane)
        local normDir to hHat.
        
        // Calculate how much radial/normal dV we need
        // Simple correction: add component toward where target will be
        local correctionScale to 0.1. // Conservative correction
        local corrVec to missVec:NORMALIZED * (missDist * correctionScale / tTrans).
        
        // Project correction onto radial and normal
        local radCorr to VDOT(corrVec, radDir).
        local normCorr to VDOT(corrVec, normDir).
        
        // Add to burn vector
        set burnVec to burnVec + (radDir * radCorr) + (normDir * normCorr).
        
        tbLog("Correction: rad=" + ROUND(radCorr, 2) + " norm=" + ROUND(normCorr, 2)).
        
        // Remove old node and add corrected one
        tbClearOwnNode.
        if not tbAddVectorNode(burnVec, dtWindow) { return false. }
    }
    
    print "Нода: eta=" + ROUND(dtWindow, 1) + " dv=" + ROUND(tbOwnNode:DELTAV:MAG, 1).
    
    if not tbExecNode() { return false. }
    
    print "Фаза 2 завершена: переходная орбита создана.".
    return true.
}

// ==========================================================================
// Phase 3: Mid-course corrections and plane alignment
// ==========================================================================
function tbPhase3 {
    print "=== ФАЗА 3: Коррекции на переходе ===".
    
    // Check if planes need alignment
    local nShip to orbitNormal(bodyPosAt(0), SHIP:VELOCITY:ORBIT).
    local tgtPos to ORBITAT(tbTarget, TIME:SECONDS):POSITION.
    local tgtVel to ORBITAT(tbTarget, TIME:SECONDS):VELOCITY:ORBIT.
    local nTarget to orbitNormal(tgtPos, tgtVel).
    local relInc to VANG(nShip, nTarget).
    
    if relInc > 1.0 {
        print "Совмещение плоскостей: relInc=" + ROUND(relInc, 2) + " deg".
        
        if not alignPlaneToNormal(nTarget, -1, 0.05, 3, true, true,
                                  0.5, tbMaxBurn, 8, 50, 10000, 30, 5)[0] {
            print "Совмещение плоскостей не выполнено.".
            return false.
        }
    }
    
    // Mid-course corrections
    local burnCount to 0.
    local deadline to TIME:SECONDS + tbMaxWinDays * 86400.
    
    until SHIP:ORBIT:BODY:NAME = tbTargetName {
        if SHIP:STATUS = "LANDED" or SHIP:STATUS = "SPLASHED" { return false. }
        if TIME:SECONDS > deadline {
            print "Таймаут SOI.".
            return false.
        }
        
        // Check if correction needed
        local relPos to bodyPosAt(t) - ORBITAT(tbTarget, TIME:SECONDS + t):POSITION.
        local relVel to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + t)):ORBIT - 
                        ORBITAT(tbTarget, TIME:SECONDS + t):VELOCITY:ORBIT.
        local dist to relPos:MAG.
        
        // Predict periapsis
        local hVec to VCRS(relPos, relVel).
        local hMag to hVec:MAG.
        if hMag > 1 {
            local energy to relVel:MAG * relVel:MAG / 2 - tbTargetMu / dist.
            local pSem to hMag * hMag / tbTargetMu.
            local eccArg to 1 + 2 * energy * pSem / tbTargetMu.
            if eccArg > 0 {
                local ecc to SQRT(eccArg).
                local rp to pSem / (1 + ecc).
                
                if rp < tbTargetRad + tbSafePeAlt - 5000 {
                    // Need correction
                    if burnCount < tbMaxMidBurns {
                        local tca to -VDOT(relPos, relVel) / (relVel:MAG * relVel:MAG).
                        if tca > 30 {
                            print "Коррекция: rp=" + ROUND(rp - tbTargetRad) + " м".
                            // Simple prograde correction
                            local corrDv to VCRS(hVec:NORMALIZED, relPos:NORMALIZED):NORMALIZED * 5.
                            if not tbAddVectorNode(corrDv, tca * 0.5) { return false. }
                            if not tbExecNode() { return false. }
                            set burnCount to burnCount + 1.
                        }
                    }
                }
            }
        }
        
        kuniverse:timewarp:warpto(TIME:SECONDS + ship:orbit:eta:transition).
        wait 0.5.
    }
    
    set WARP to 0.
    print "Фаза 3 завершена: вход в SOI " + tbTargetName + ".".
    return true.
}

// ==========================================================================
// Phase 4: Capture at target
// ==========================================================================
function tbPhase4 {
    print "=== ФАЗА 4: Захват на орбиту цели ===".
    
    // Find periapsis time
    local scanLimit to MAX(600, tbTargetPeriod * 2).
    local step to MAX(10, scanLimit / 60).
    local bestT to 0.
    local bestDist to (SHIP:POSITION - tbTarget:POSITION):MAG.
    local t to step.
    
    until t > scanLimit {
        local shipPos to bodyPosAt(t).
        local tgtPos to ORBITAT(tbTarget, TIME:SECONDS + t):POSITION.

        local dist to (shipPos - tgtPos):MAG.
        
        if dist < bestDist {
            set bestDist to dist.
            set bestT to t.
        }
        set t to t + step.
        wait 0.
    }
    
    // Refine
    local lo to MAX(0, bestT - step).
    local hi to bestT + step.
    local i to 0.
    until i >= 15 or (hi - lo) < 1 {
        local m1 to lo + (hi - lo) / 3.
        local m2 to hi - (hi - lo) / 3.
        
        local shipPos1 to POSITIONAT(SHIP, TIMESTAMP(TIME:SECONDS + m1)).
        local tgtPos1 to ORBITAT(tbTarget, TIME:SECONDS + m1):POSITION.
        local dist1 to (shipPos1 - tgtPos1):MAG.
        
        local shipPos2 to POSITIONAT(SHIP, TIMESTAMP(TIME:SECONDS + m2)).
        local tgtPos2 to ORBITAT(tbTarget, TIME:SECONDS + m2):POSITION.
        local dist2 to (shipPos2 - tgtPos2):MAG.
        
        if dist1 < dist2 {
            set hi to m2.
        } else {
            set lo to m1.
        }
        set i to i + 1.
    }
    
    local periEta to (lo + hi) / 2.
    local periRad to bestDist.
    
    if periRad <= tbTargetRad + 100 {
        print "Перицентр ниже поверхности.".
        return false.
    }
    
    // Calculate capture burn
    local periVel to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + periEta)):ORBIT:MAG.
    local aNew to (periRad + tbTargetRad + tbDestApAlt) / 2.
    local vReq to SQRT(MAX(0, tbTargetMu * (2 / periRad - 1 / aNew))).
    local capDv to vReq - periVel.
    
    print "Захват: eta=" + ROUND(periEta, 1) + " rpAlt=" + ROUND(periRad - tbTargetRad) + " dV=" + ROUND(capDv, 1).
    
    if ABS(capDv) > 0.5 {
        if ABS(capDv) > tbMaxBurn {
            print "dV захвата велик.".
            return false.
        }
        
        local capVel to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + periEta)):ORBIT.
        local capVec to capDv * capVel:NORMALIZED.
        
        if not tbAddVectorNode(capVec, periEta) { return false. }
        if not tbExecNode() { return false. }
    }
    
    if SHIP:ORBIT:ECCENTRICITY >= 1 {
        print "Орбита не замкнута.".
        return false.
    }
    
    print "Фаза 4 завершена: захват выполнен.".
    return true.
}

// ==========================================================================
// Main function
// ==========================================================================
function tbMain {
    print "=== TRANSFER TO BODY ===".
    
    set WARP to 0.
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    tbClearOwnNode.
    
    if HASNODE {
        print "Есть чужая маневровая нода. Удали её вручную.".
        return false.
    }
    
    // Setup target
    if targetName <> "" {
        set target to targetName.
    }
    
    if SHIP:MAXTHRUST <= 0.01 {
        print "Нет тяги.".
        return false.
    }
    
    if not HASTARGET {
        print "Нет цели (TARGET).".
        return false.
    }
    
    set tbTarget to TARGET.
    set tbTargetName to tbTarget:NAME.
    set tbCentralName to tbTarget:ORBIT:BODY:NAME.
    set tbCentralMu to tbTarget:ORBIT:BODY:MU.
    set tbTargetMu to tbTarget:MU.
    set tbTargetRad to tbTarget:RADIUS.
    set tbTargetSMA to tbTarget:ORBIT:SEMIMAJORAXIS.
    set tbTargetPeriod to MAX(60, tbTarget:ORBIT:PERIOD).
    
    if tbCentralMu <= 0 or tbTargetMu <= 0 or tbTargetRad <= 0 or tbTargetSMA <= 1 {
        print "Некорректные параметры тел.".
        return false.
    }
    
    set tbSafePeAlt to tbGetSafePeAlt(tbTarget).
    
    if tbDestApAlt < tbDestPeAlt {
        set tbDestApAlt to tbDestApAlt + tbDestPeAlt.
        set tbDestPeAlt to tbDestApAlt - tbDestPeAlt.
        set tbDestApAlt to tbDestApAlt - tbDestPeAlt.
    }
    
    set tbDestPeAlt to MAX(tbDestPeAlt, tbSafePeAlt).
    set tbDestApAlt to MAX(tbDestApAlt, tbDestPeAlt).
    
    print "Цель: " + tbTargetName.
    print "Орбита: Pe=" + ROUND(tbDestPeAlt) + " Ap=" + ROUND(tbDestApAlt).
    
    // Detect and execute phases
    local phase to tbDetectPhase().
    print "Текущая фаза: " + phase.
    
    if phase = 1 {
        if not tbPhase1() { return false. }
        set phase to tbDetectPhase().
    }
    
    if phase = 2 {
        if not tbPhase2() { return false. }
        set phase to 3.
    }
    
    if phase = 3 {
        if not tbPhase3() { return false. }
        set phase to 4.
    }
    
    if phase = 4 {
        if not tbPhase4() { return false. }
    }
    
    if phase = 5 {
        print "Уже на орбите цели.".
    }
    
    // Final orbit circularization
    run orbit_to(tbDestApAlt, tbDestPeAlt).
    
    print "=== ПЕРЕХОД ЗАВЕРШЁН ===".
    print "Тело: " + SHIP:ORBIT:BODY:NAME.
    print "Ap=" + ROUND(SHIP:ORBIT:APOAPSIS) + " Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS).
    
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    return true.
}

tbMain().