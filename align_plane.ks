@lazyglobal on.

parameter libraryOnly is false.

parameter tolDeg is 0.05.
parameter maxIters is 3.
parameter minTimeSec is 30.

parameter minBurnDv is 0.5.
parameter maxBurnDv is 10000.

parameter maxNodeMisalignDeg is 8.
parameter maxSpeedErr is 2.0.
parameter maxApPeDrift is 10000.
parameter maxNodeErrDeg is 5.

run utils.

set apHasNode to false.

function apDropNode {
    if apHasNode {
        remove apNode.
        set apHasNode to false.
    }
}

function apPrepareForNode {
    apDropNode.

    if HASNODE {
        print "align: есть чужая нода. Удали её вручную.".
        return false.
    }

    return true.
}

function apAddNode { parameter etaSec.
    if not apPrepareForNode {
        return false.
    }

    local etaVal to MAX(5, etaSec).

    set apNode to NODE(TIME:SECONDS + etaVal, 0, 1, 0).
    add apNode.
    set apHasNode to true.

    wait 0.3.

    if not HASNODE {
        set apHasNode to false.
        return false.
    }

    return true.
}

function apCalibrateAxes {
    local zeroList to LIST(V(0, 0, 0), V(0, 0, 0), V(0, 0, 0)).

    if not apHasNode {
        return zeroList.
    }

    set apNode:RADIALOUT to 0.
    set apNode:NORMAL to 0.
    set apNode:PROGRADE to 0.
    wait 0.05.

    set apNode:NORMAL to 1.
    wait 0.05.

    local realNormal to apNode:DELTAV.
    if not vecIsOk(realNormal) or realNormal:MAG < 0.000001 {
        return zeroList.
    }
    set realNormal to realNormal:NORMALIZED.

    set apNode:NORMAL to 0.
    set apNode:PROGRADE to 1.
    wait 0.05.

    local realPrograde to apNode:DELTAV.
    if not vecIsOk(realPrograde) or realPrograde:MAG < 0.000001 {
        return zeroList.
    }
    set realPrograde to realPrograde:NORMALIZED.

    set apNode:PROGRADE to 0.
    set apNode:RADIALOUT to 1.
    wait 0.05.

    local realRadial to apNode:DELTAV.
    if not vecIsOk(realRadial) or realRadial:MAG < 0.000001 {
        return zeroList.
    }
    set realRadial to realRadial:NORMALIZED.

    return LIST(realRadial, realNormal, realPrograde).
}

function apNormalOf { parameter orbVsl.
    if orbVsl:NAME = SHIP:NAME {
        return orbitNormal(bodyPosAt(0), SHIP:VELOCITY:ORBIT).
    }

    local posVec to ORBITAT(orbVsl, TIME:SECONDS):POSITION.
    local velVec to ORBITAT(orbVsl, TIME:SECONDS):VELOCITY:ORBIT.

    return orbitNormal(posVec, velVec).
}

function apAcuteNormal { parameter refNrm. parameter tgtNrm.
    local nVec to tgtNrm.

    if not vecIsOk(nVec) or nVec:MAG < 0.5 {
        return refNrm:NORMALIZED.
    }

    set nVec to nVec:NORMALIZED.

    if VANG(refNrm, nVec) > 90 {
        set nVec to (-1) * nVec.
    }

    return nVec.
}

function apNodeErrDeg { parameter posVec. parameter nA. parameter nB.
    local lineVec to VCRS(nA, nB).

    if not vecIsOk(lineVec) or lineVec:MAG < 0.000001 {
        return 0.
    }

    if not vecIsOk(posVec) or posVec:MAG < 1 {
        return 999.
    }

    set lineVec to lineVec:NORMALIZED.

    local cosVal to ABS(VDOT(posVec:NORMALIZED, lineVec)).
    set cosVal to clamp(cosVal, 0, 1).

    return ARCCOS(cosVal).
}

function apPlaneDvCandidate { parameter rPre. parameter vPre. parameter nTgt. parameter signDir.
    local zeroVec to V(0, 0, 0).
    local badRet to LIST(zeroVec, 999).

    local rMag to rPre:MAG.
    local vMag to vPre:MAG.

    if rMag < 1 or vMag < 0.01 {
        return badRet.
    }

    local rHat to rPre:NORMALIZED.
    local hPre to orbitNormal(rPre, vPre).
    local nUse to apAcuteNormal(hPre, nTgt).

    local nProj to nUse - (VDOT(nUse, rHat) * rHat).

    if not vecIsOk(nProj) or nProj:MAG < 0.000001 {
        return badRet.
    }

    set nProj to nProj:NORMALIZED.

    local alphaDeg to VANG(hPre, nProj).

    if alphaDeg < 0.001 {
        return LIST(zeroVec, alphaDeg).
    }

    local vrScalar to VDOT(vPre, rHat).
    local vTanVec to vPre - (vrScalar * rHat).

    if not vecIsOk(vTanVec) or vTanVec:MAG < 0.01 {
        return badRet.
    }

    local vRotVec to (vrScalar * rHat) + (ANGLEAXIS(signDir * alphaDeg, rHat) * vTanVec).
    local dvVec to vRotVec - vPre.

    local hNew to orbitNormal(rPre, vRotVec).
    local errDeg to 999.

    if vecIsOk(hNew) and hNew:MAG > 0.000001 {
        set errDeg to VANG(hNew, nUse).
    }

    return LIST(dvVec, errDeg).
}

function alignPlaneToNormal {
    parameter tgtNrm.
    parameter etaIn.
    parameter tolVal.
    parameter iterLim.
    parameter execNow.
    parameter preserve.
    parameter minDv.
    parameter maxDv.
    parameter maxMis.
    parameter maxSpd.
    parameter maxDrift.
    parameter minTime.
    parameter maxNodeErr.

    local badRet to LIST(false, 0, 999, 0).

    if not vecIsOk(tgtNrm) or tgtNrm:MAG < 0.5 {
        return badRet.
    }

    set tgtNrm to tgtNrm:NORMALIZED.

    apDropNode.

    if HASNODE {
        print "align: есть чужая нода. Удали её вручную.".
        return badRet.
    }

    local nShip to apNormalOf(SHIP).
    set tgtNrm to apAcuteNormal(nShip, tgtNrm).

    local periodVal to MAX(60, SHIP:ORBIT:PERIOD).
    local minEta to MAX(30, minTime).

    local etaVal to etaIn.

    if not isFiniteVal(etaVal) or etaVal < 0 {
        set etaVal to findPlaneCrossTime(tgtNrm, minEta, periodVal).
    }

    if not isFiniteVal(etaVal) or etaVal < 5 {
        return badRet.
    }

    local halfP to periodVal / 2.
    local bestEta to etaVal.
    local bestNodeErr to 999.

    local mIdx to -2.

    until mIdx > 2 {
        local candEta to etaVal + mIdx * halfP.

        if candEta >= minEta {
            local candPos to bodyPosAt(candEta).
            local candVel to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + candEta)):ORBIT.

            if vecIsOk(candPos) and vecIsOk(candVel) {
                local candNrm to orbitNormal(candPos, candVel).
                local candErr to apNodeErrDeg(candPos, candNrm, tgtNrm).

                if candErr < bestNodeErr {
                    set bestNodeErr to candErr.
                    set bestEta to candEta.
                }
            }
        }

        set mIdx to mIdx + 1.
        wait 0.
    }

    set etaVal to bestEta.

    if bestNodeErr > maxNodeErr {
        return LIST(false, 0, bestNodeErr, etaVal).
    }

    local iterIdx to 0.
    local dvMag to 0.
    local errDeg to 999.

    until iterIdx >= iterLim {
        if not apAddNode(etaVal) {
            return LIST(false, dvMag, errDeg, etaVal).
        }

        local rPre to bodyPosAt(etaVal).
        local vPre to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + etaVal)):ORBIT.

        if not vecIsOk(rPre) or not vecIsOk(vPre) {
            apDropNode.
            return LIST(false, dvMag, errDeg, etaVal).
        }

        if rPre:MAG < 1 or vPre:MAG < 0.01 {
            apDropNode.
            return LIST(false, dvMag, errDeg, etaVal).
        }

        local hPre to orbitNormal(rPre, vPre).
        local nLocal to apAcuteNormal(hPre, tgtNrm).

        set errDeg to VANG(hPre, nLocal).

        if errDeg <= tolVal {
            apDropNode.
            return LIST(true, 0, errDeg, etaVal).
        }

        local axesList to apCalibrateAxes().

        if not vecIsOk(axesList[0]) or not vecIsOk(axesList[1]) or not vecIsOk(axesList[2]) {
            apDropNode.
            return LIST(false, dvMag, errDeg, etaVal).
        }

        local hasBest to false.

        local bestRad to 0.
        local bestNorm to 0.
        local bestPro to 0.
        local bestMis to 999.
        local bestSpd to 999.
        local bestPred to 999.

        local preAp to SHIP:ORBIT:APOAPSIS.
        local prePe to SHIP:ORBIT:PERIAPSIS.

        local signIdx to 0.

        until signIdx >= 2 {
            local signDir to 1.

            if signIdx = 1 {
                set signDir to -1.
            }

            local candList to apPlaneDvCandidate(rPre, vPre, nLocal, signDir).
            local candDv to candList[0].

            local tmpRad to VDOT(candDv, axesList[0]).
            local tmpNorm to VDOT(candDv, axesList[1]).
            local tmpPro to VDOT(candDv, axesList[2]).

            if preserve {
                set tmpRad to 0.
            }

            set apNode:RADIALOUT to tmpRad.
            set apNode:NORMAL to tmpNorm.
            set apNode:PROGRADE to tmpPro.
            wait 0.1.

            local nodeDv to apNode:DELTAV.
            local tmpMis to 999.

            if nodeDv:MAG > 0.000001 {
                set tmpMis to VANG(candDv, nodeDv).
            }

            local predV to vPre + nodeDv.
            local predH to orbitNormal(rPre, predV).
            local tmpPred to 999.

            if vecIsOk(predH) and predH:MAG > 0.000001 {
                set tmpPred to VANG(predH, nLocal).
            }

            local tmpSpd to ABS(predV:MAG - vPre:MAG).
            local driftOk to true.

            if preserve {
                local postOrb to apNode:ORBIT.

                if ABS(postOrb:APOAPSIS - preAp) > maxDrift {
                    set driftOk to false.
                }

                if ABS(postOrb:PERIAPSIS - prePe) > maxDrift {
                    set driftOk to false.
                }
            }

            if tmpMis <= maxMis and tmpSpd <= maxSpd and driftOk and tmpPred < bestPred {
                set bestRad to tmpRad.
                set bestNorm to tmpNorm.
                set bestPro to tmpPro.
                set bestMis to tmpMis.
                set bestSpd to tmpSpd.
                set bestPred to tmpPred.
                set hasBest to true.
            }

            set signIdx to signIdx + 1.
        }

        if not hasBest {
            apDropNode.
            return LIST(false, dvMag, errDeg, etaVal).
        }

        if bestPred >= errDeg {
            apDropNode.
            return LIST(false, dvMag, errDeg, etaVal).
        }

        set apNode:RADIALOUT to bestRad.
        set apNode:NORMAL to bestNorm.
        set apNode:PROGRADE to bestPro.
        wait 0.2.

        set dvMag to apNode:DELTAV:MAG.
        set errDeg to bestPred.

        if dvMag < minDv {
            apDropNode.
            return LIST(true, 0, errDeg, etaVal).
        }

        if dvMag > maxDv {
            apDropNode.
            return LIST(false, dvMag, errDeg, etaVal).
        }

        if execNow {
            unlock THROTTLE.
            unlock STEERING.

            run exec_node(apNode).

            set apHasNode to false.
            wait 1.

            local nAfter to apNormalOf(SHIP).
            local nAfterTgt to apAcuteNormal(nAfter, tgtNrm).

            set errDeg to VANG(nAfter, nAfterTgt).

            if errDeg <= tolVal {
                return LIST(true, dvMag, errDeg, etaVal).
            }

            if etaIn >= 0 {
                return LIST(false, dvMag, errDeg, etaVal).
            }

            set etaVal to findPlaneCrossTime(
                nAfterTgt,
                MAX(minEta, etaVal + 10),
                MAX(60, SHIP:ORBIT:PERIOD)
            ).

            if not isFiniteVal(etaVal) or etaVal < 5 {
                return LIST(false, dvMag, errDeg, etaVal).
            }
        } else {
            apDropNode.
            return LIST(errDeg <= tolVal, dvMag, errDeg, etaVal).
        }

        set iterIdx to iterIdx + 1.
        wait 0.
    }

    apDropNode.
    return LIST(false, dvMag, errDeg, etaVal).
}

if not libraryOnly {
    print "=== ALIGN PLANE ===".

    apDropNode.

    local canRun to true.

    if HASNODE {
        print "Есть чужая маневровая нода. Удали её вручную.".
        set canRun to false.
    }

    if canRun and not HASTARGET {
        print "NO TARGET".
        set canRun to false.
    }

    if canRun and SHIP:MAXTHRUST <= 0.01 {
        print "NO THRUST".
        set canRun to false.
    }

    if canRun and SHIP:ORBIT:ECCENTRICITY >= 1 {
        print "OPEN ORBIT".
        set canRun to false.
    }

    if canRun {
        local tgtNrm to apNormalOf(TARGET).

        local resList to alignPlaneToNormal(
            tgtNrm,
            -1,
            tolDeg,
            maxIters,
            true,
            true,
            minBurnDv,
            maxBurnDv,
            maxNodeMisalignDeg,
            maxSpeedErr,
            maxApPeDrift,
            minTimeSec,
            maxNodeErrDeg
        ).

        print "ok=" + resList[0] +
              " dv=" + ROUND(resList[1], 2) +
              " err=" + ROUND(resList[2], 4) +
              " eta=" + ROUND(resList[3], 1).
    }
}