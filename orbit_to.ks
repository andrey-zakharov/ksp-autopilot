// ==========================================================================
// orbit_to.ks
// Перевод на орбиту с заданными Ap, Pe, наклонением.
// RUN orbit_to(apAlt).
// RUN orbit_to(apAlt, peAlt).
// RUN orbit_to(apAlt, peAlt, incDeg).
// Высоты — над уровнем моря.
// ==========================================================================
PARAMETER apAlt IS 0.
PARAMETER peAlt IS -1.
PARAMETER incDeg IS 0.
PARAMETER tolM IS 5.
PARAMETER tolDeg IS 0.05.
PARAMETER maxDV IS 10000.

RUN utils.

SET otTolM TO tolM.
SET otTolDeg TO tolDeg.
SET otMaxDV TO maxDV.

// ==========================================================================
// Вспомогательные функции
// ==========================================================================

FUNCTION otMinEta { PARAMETER dvM.
    LOCAL m TO MAX(0.001, SHIP:MASS).
    LOCAL a TO MAX(0.1, SHIP:MAXTHRUST / m).
    RETURN MAX(30, estimateTurnTime(180, m) + dvM / a / 2 + 10).
}

FUNCTION otPushEta { PARAMETER eta. PARAMETER minE.
    LOCAL x TO eta.
    IF NOT isFiniteVal(x) {
        RETURN minE.
    }
    IF x < minE {
        LOCAL p TO MAX(60, SHIP:ORBIT:PERIOD).
        LOCAL e TO SHIP:ORBIT:ECCENTRICITY.
        IF isFiniteVal(e) AND e > 0.0001 {
            UNTIL x >= minE {
                SET x TO x + p.
            }
        } ELSE {
            SET x TO minE.
        }
    }
    RETURN x.
}

FUNCTION otMeanDeg {
    LOCAL e TO SHIP:ORBIT:ECCENTRICITY.
    IF NOT isFiniteVal(e) OR e < 0.00001 {
        RETURN 0.
    }
    LOCAL nu TO normDeg360(SHIP:ORBIT:TRUEANOMALY).
    LOCAL k TO SQRT(MAX(0.000000000001, (1 - e) / (1 + e))).
    LOCAL eAn TO 2 * ARCTAN2(k * SIN(nu / 2), COS(nu / 2)).
    RETURN normDeg360(eAn - (180 / CONSTANT:PI) * e * SIN(eAn)).
}

FUNCTION otEtaApsis { PARAMETER toPe.
    LOCAL e TO SHIP:ORBIT:ECCENTRICITY.
    LOCAL p TO MAX(60, SHIP:ORBIT:PERIOD).
    IF NOT isFiniteVal(e) OR e < 0.00001 {
        RETURN 0.
    }

    LOCAL m TO otMeanDeg.
    LOCAL tgt TO 0.
    IF NOT toPe {
        SET tgt TO 180.
    }

    LOCAL d TO normDeg360(tgt - m).
    IF d < 0.000001 {
        SET d TO 0.
    }

    RETURN d / 360 * p.
}

FUNCTION otNorth {
    LOCAL n TO BODY:ANGULARVEL.
    IF NOT vecIsOk(n) OR n:MAG < 0.00000001 {
        RETURN V(0, 1, 0).
    }
    RETURN n:NORMALIZED.
}

FUNCTION otCurrNormal {
    RETURN orbitNormal(bodyPosAt(0), SHIP:VELOCITY:ORBIT).
}

FUNCTION otTargetNormal { PARAMETER incW.
    LOCAL nH TO otNorth.
    LOCAL hH TO otCurrNormal.

    LOCAL prV TO hH - (VDOT(hH, nH) * nH).
    IF NOT vecIsOk(prV) OR prV:MAG < 0.000001 {
        LOCAL pV TO bodyPosAt(0).
        SET prV TO pV - (VDOT(pV, nH) * nH).
    }
    IF NOT vecIsOk(prV) OR prV:MAG < 0.000001 {
        LOCAL axV TO V(1, 0, 0).
        SET prV TO axV - (VDOT(axV, nH) * nH).
    }
    IF NOT vecIsOk(prV) OR prV:MAG < 0.000001 {
        RETURN nH.
    }
    SET prV TO prV:NORMALIZED.

    LOCAL tN TO nH * COS(incW) + prV * SIN(incW).
    IF NOT vecIsOk(tN) OR tN:MAG < 0.000001 {
        RETURN nH.
    }
    RETURN tN:NORMALIZED.
}

FUNCTION otPlaneDv { PARAMETER eta. PARAMETER tN.
    LOCAL pV TO bodyPosAt(eta).
    LOCAL vV TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + eta)):ORBIT.

    IF NOT vecIsOk(pV) OR NOT vecIsOk(vV) OR pV:MAG < 1 OR vV:MAG < 1 {
        RETURN V(0, 0, 0).
    }

    LOCAL hV TO orbitNormal(pV, vV).
    LOCAL ang TO VANG(hV, tN).
    IF ang <= otTolDeg {
        RETURN V(0, 0, 0).
    }

    LOCAL rH TO pV:NORMALIZED.

    LOCAL vA TO ANGLEAXIS(ang, rH) * vV.
    LOCAL vB TO ANGLEAXIS((-1) * ang, rH) * vV.

    LOCAL hA TO orbitNormal(pV, vA).
    LOCAL hB TO orbitNormal(pV, vB).

    LOCAL eA TO VANG(hA, tN).
    LOCAL eB TO VANG(hB, tN).

    LOCAL vW TO vA.
    IF eB < eA {
        SET vW TO vB.
    }

    RETURN vW - vV.
}

FUNCTION otExecNode { PARAMETER eta. PARAMETER dvV.
    LOCAL dvM TO dvV:MAG.
    IF NOT isFiniteVal(dvM) OR dvM < 0.5 {
        RETURN TRUE.
    }
    IF dvM > otMaxDV {
        RETURN FALSE.
    }

    UNTIL NOT HASNODE {
        REMOVE NEXTNODE.
    }

    LOCAL otMnv TO NODE(TimeSpan(eta), 0, 1, 0).
    ADD otMnv.
    WAIT 0.3.

    IF NOT HASNODE {
        RETURN FALSE.
    }

    LOCAL ax TO calibrateNodeAxes().
    LOCAL rA TO ax[0].
    LOCAL nA TO ax[1].
    LOCAL pA TO ax[2].

    IF NOT vecIsOk(nA) OR nA:MAG < 0.5 {
        REMOVE NEXTNODE.
        RETURN FALSE.
    }

    SET NEXTNODE:RADIALOUT TO VDOT(dvV, rA).
    SET NEXTNODE:NORMAL TO VDOT(dvV, nA).
    SET NEXTNODE:PROGRADE TO VDOT(dvV, pA).
    WAIT 0.2.

    IF VANG(NEXTNODE:DELTAV, dvV) > 5.0 {
        REMOVE NEXTNODE.
        RETURN FALSE.
    }

    UNLOCK THROTTLE.
    UNLOCK STEERING.
    RUN exec_node.
    SET SAS TO FALSE.

    UNTIL NOT HASNODE {
        REMOVE NEXTNODE.
    }

    RETURN TRUE.
}

// ==========================================================================
// Постановка апоцентра: импульс в перицентре
// ==========================================================================

FUNCTION otSetAp { PARAMETER rW.
    LOCAL rAp TO BODY:RADIUS + SHIP:ORBIT:APOAPSIS.
    LOCAL rPe TO BODY:RADIUS + SHIP:ORBIT:PERIAPSIS.

    IF NOT isFiniteVal(rAp) OR NOT isFiniteVal(rPe) OR rPe <= BODY:RADIUS + 1 {
        RETURN FALSE.
    }

    IF ABS(rAp - rW) <= otTolM {
        RETURN TRUE.
    }

    IF rW < rPe - otTolM {
        RETURN FALSE.
    }

    LOCAL muV TO BODY:MU.
    LOCAL aC TO SHIP:ORBIT:SEMIMAJORAXIS.
    LOCAL aN TO (rW + rPe) / 2.

    IF NOT isFiniteVal(muV) OR muV <= 0 {
        RETURN FALSE.
    }
    IF NOT isFiniteVal(aC) OR aC <= 0 {
        RETURN FALSE.
    }
    IF NOT isFiniteVal(aN) OR aN <= 0 {
        RETURN FALSE.
    }

    LOCAL vC TO SQRT(MAX(0, muV * (2 / rPe - 1 / aC))).
    LOCAL vN TO SQRT(MAX(0, muV * (2 / rPe - 1 / aN))).
    LOCAL dvS TO vN - vC.

    IF ABS(dvS) < 0.5 {
        RETURN TRUE.
    }

    LOCAL eta TO otEtaApsis(TRUE).
    SET eta TO otPushEta(eta, otMinEta(ABS(dvS))).

    LOCAL vV TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + eta)):ORBIT.
    IF NOT vecIsOk(vV) OR vV:MAG < 1 {
        RETURN FALSE.
    }

    RETURN otExecNode(eta, dvS * vV:NORMALIZED).
}

// ==========================================================================
// Постановка перицентра: импульс в апоцентре
// ==========================================================================

FUNCTION otSetPe { PARAMETER rW.
    LOCAL rAp TO BODY:RADIUS + SHIP:ORBIT:APOAPSIS.
    LOCAL rPe TO BODY:RADIUS + SHIP:ORBIT:PERIAPSIS.

    IF NOT isFiniteVal(rAp) OR NOT isFiniteVal(rPe) OR rAp <= BODY:RADIUS + 1 {
        RETURN FALSE.
    }

    IF ABS(rPe - rW) <= otTolM {
        RETURN TRUE.
    }

    IF rW > rAp + otTolM {
        RETURN FALSE.
    }

    LOCAL muV TO BODY:MU.
    LOCAL aC TO SHIP:ORBIT:SEMIMAJORAXIS.
    LOCAL aN TO (rAp + rW) / 2.

    IF NOT isFiniteVal(muV) OR muV <= 0 {
        RETURN FALSE.
    }
    IF NOT isFiniteVal(aC) OR aC <= 0 {
        RETURN FALSE.
    }
    IF NOT isFiniteVal(aN) OR aN <= 0 {
        RETURN FALSE.
    }

    LOCAL vC TO SQRT(MAX(0, muV * (2 / rAp - 1 / aC))).
    LOCAL vN TO SQRT(MAX(0, muV * (2 / rAp - 1 / aN))).
    LOCAL dvS TO vN - vC.

    IF ABS(dvS) < 0.5 {
        RETURN TRUE.
    }

    LOCAL eta TO otEtaApsis(FALSE).
    SET eta TO otPushEta(eta, otMinEta(ABS(dvS))).

    LOCAL vV TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + eta)):ORBIT.
    IF NOT vecIsOk(vV) OR vV:MAG < 1 {
        RETURN FALSE.
    }

    RETURN otExecNode(eta, dvS * vV:NORMALIZED).
}

// ==========================================================================
// Смена наклонения
// ==========================================================================

FUNCTION otPlane { PARAMETER incW.
    LOCAL curI TO SHIP:ORBIT:INCLINATION.
    IF NOT isFiniteVal(curI) {
        RETURN FALSE.
    }

    IF ABS(curI - incW) <= otTolDeg {
        RETURN TRUE.
    }

    LOCAL tN TO otTargetNormal(incW).
    LOCAL hH TO otCurrNormal.

    IF VANG(hH, tN) <= otTolDeg {
        RETURN TRUE.
    }

    LOCAL m TO MAX(0.001, SHIP:MASS).
    LOCAL minT TO MAX(30, estimateTurnTime(90, m) + 10).
    LOCAL p TO MAX(60, SHIP:ORBIT:PERIOD).

    LOCAL eta TO findPlaneCrossTime(tN, minT, p).
    IF NOT isFiniteVal(eta) OR eta < 5 {
        RETURN FALSE.
    }

    LOCAL dvV TO otPlaneDv(eta, tN).
    LOCAL dvM TO dvV:MAG.

    IF dvM < 0.5 {
        RETURN TRUE.
    }
    IF dvM > otMaxDV {
        RETURN FALSE.
    }

    SET eta TO otPushEta(eta, otMinEta(dvM)).
    RETURN otExecNode(eta, dvV).
}

// ==========================================================================
// Старт
// ==========================================================================

PRINT "=== ORBIT_TO ===".
SET WARP TO 0.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

UNTIL NOT HASNODE {
    REMOVE NEXTNODE.
}

IF SHIP:MAXTHRUST <= 0.01 {
    PRINT "Нет тяги.".
    EXIT.
}

IF NOT isFiniteVal(SHIP:ORBIT:APOAPSIS) OR
   NOT isFiniteVal(SHIP:ORBIT:PERIAPSIS) OR
   SHIP:ORBIT:PERIAPSIS <= 0 OR
   SHIP:ORBIT:ECCENTRICITY >= 1 {
    PRINT "Нет замкнутой орбиты.".
    EXIT.
}

LOCAL apW TO apAlt.
IF NOT isFiniteVal(apW) OR apW <= 0 {
    SET apW TO SHIP:ORBIT:APOAPSIS.
}

LOCAL peW TO peAlt.
IF NOT isFiniteVal(peW) OR peW < 0 {
    SET peW TO apW.
}

IF apW < 10 {
    SET apW TO 10.
}
IF peW < 10 {
    SET peW TO 10.
}
IF peW > apW {
    SET peW TO apW.
}

LOCAL incW TO clamp(incDeg, 0, 179.9).

LOCAL rApW TO BODY:RADIUS + apW.
LOCAL rPeW TO BODY:RADIUS + peW.

PRINT "Цель: Ap=" + ROUND(apW) +
      " Pe=" + ROUND(peW) +
      " inc=" + ROUND(incW, 3).

LOCAL curApR TO BODY:RADIUS + SHIP:ORBIT:APOAPSIS.
LOCAL okF TO TRUE.

// Наклонение меняем на более высокой орбите.
IF rApW > curApR + otTolM {
    PRINT "План: подъём Ap -> наклонение -> Pe.".

    SET okF TO otSetAp(rApW).
    IF okF {
        SET okF TO otPlane(incW).
    }
    IF okF {
        SET okF TO otSetPe(rPeW).
    }
} ELSE {
    PRINT "План: наклонение -> Pe -> Ap.".

    SET okF TO otPlane(incW).
    IF okF {
        SET okF TO otSetPe(rPeW).
    }
    IF okF {
        SET okF TO otSetAp(rApW).
    }
}

// Мелкая доводка.
IF okF {
    LOCAL nowAp TO BODY:RADIUS + SHIP:ORBIT:APOAPSIS.
    IF ABS(nowAp - rApW) > otTolM * 3 {
        SET okF TO otSetAp(rApW).
    }
}

IF okF {
    LOCAL nowPe TO BODY:RADIUS + SHIP:ORBIT:PERIAPSIS.
    IF ABS(nowPe - rPeW) > otTolM * 3 {
        SET okF TO otSetPe(rPeW).
    }
}

IF okF {
    PRINT "=== ГОТОВО ===".
    PRINT "Ap=" + ROUND(SHIP:ORBIT:APOAPSIS) +
          " Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) +
          " inc=" + ROUND(SHIP:ORBIT:INCLINATION, 3).
} ELSE {
    PRINT "=== ОТМЕНА ===".
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.