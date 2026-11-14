// ==========================================================================
// suicide_burn.ks — финальное торможение с непрерывным наведением на цель
// ==========================================================================
PARAMETER clearance IS 30.
PARAMETER exitSpeed IS 10.
PARAMETER maxTiltDeg IS 8.
PARAMETER touchdownSpeed IS 0.
PARAMETER maxIgniteTiltDeg IS 15.
PARAMETER slewDeg IS 10.

RUN utils.
RUNONCEPATH("orbit_lib").

GLOBAL sbThrottle IS 0.
GLOBAL sbSteer IS SHIP:FACING.
GLOBAL sbDiagTimer IS 0.
GLOBAL sbPrevVVert IS 0.
GLOBAL sbAccMeas IS 0.
GLOBAL sbLastSec IS TIME:SECONDS.
GLOBAL sbAlt IS MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
GLOBAL sbVVert IS VDOT(SHIP:VELOCITY:SURFACE, SHIP:UP:FOREVECTOR).
SET sbPrevVVert TO sbVVert.

LOCK THROTTLE TO sbThrottle.
LOCK STEERING TO sbSteer.

UNTIL SHIP:STATUS = "LANDED" OR
      SHIP:STATUS = "SPLASHED" OR
      sbAlt <= clearance OR sbVVert > 0 OR
      sbAlt <= 1.0 {

    SET sbNow TO TIME:SECONDS.
    SET sbDt TO clamp(sbNow - sbLastSec, 0.01, 0.25).
    SET sbLastSec TO sbNow.
    SET sbAlt TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).

    LOCAL sbUpVec IS SHIP:UP:FOREVECTOR.
    LOCAL sbVel IS SHIP:VELOCITY:SURFACE.
    SET sbVVert TO VDOT(sbVel, sbUpVec).
    LOCAL sbVHoriz IS sbVel - (sbVVert * sbUpVec).
    LOCAL sbVHorizMag IS sbVHoriz:MAG.
    LOCAL sbVVertAbs IS ABS(sbVVert).

    LOCAL sbMass IS MAX(0.001, SHIP:MASS).
    LOCAL sbMaxThrust IS SHIP:MAXTHRUST.
    IF sbMaxThrust <= 0.01 { SET sbMaxThrust TO SHIP:AVAILABLETHRUST. }
    LOCAL sbMaxAcc IS MAX(0.01, sbMaxThrust / sbMass).
    LOCAL sbG IS safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).

    LOCAL sbTiltDeg IS 0.
    LOCAL sbDesired IS sbUpVec.

    LOCAL hasTgt IS (defined siteLat) AND (defined siteLng) AND (defined siteTerrain).

    IF hasTgt {
        LOCAL tgtGeo IS BODY:GEOPOSITIONLATLNG(siteLat, siteLng).
        LOCAL tgtPos IS tgtGeo:POSITION - BODY:POSITION.

        IF vecIsOk(tgtPos) AND tgtPos:MAG > 1 {
            LOCAL toTgt IS tgtPos - (VDOT(tgtPos, sbUpVec) * sbUpVec).
            LOCAL toTgtMag IS toTgt:MAG.

            LOCAL tGround IS 9999.
            IF sbVVert < -0.1 {
                SET tGround TO sbAlt / MAX(0.1, sbVVertAbs).
            }

            LOCAL predHoriz IS sbVHoriz * MIN(tGround, 300).
            LOCAL missVec IS toTgt - predHoriz.
            LOCAL missMag IS missVec:MAG.

            LOCAL horizCmd IS V(0,0,0).
            IF sbVHorizMag > 0.5 {
                SET horizCmd TO horizCmd + ((-1) * sbVHoriz:NORMALIZED).
            }
            IF missMag > 5 AND vecIsOk(missVec) {
                SET horizCmd TO horizCmd + (missVec:NORMALIZED * MIN(1, missMag / 500)).
            }

            IF vecIsOk(horizCmd) AND horizCmd:MAG > 0.01 {
                SET horizCmd TO horizCmd:NORMALIZED.

                LOCAL wantTilt IS 0.
                IF missMag > 5 {
                    SET wantTilt TO wantTilt + MIN(maxTiltDeg, missMag / MAX(10, sbAlt) * 45).
                }
                IF sbVHorizMag > 0.5 {
                    SET wantTilt TO wantTilt + MIN(maxTiltDeg, sbVHorizMag * 0.5).
                }
                SET sbTiltDeg TO MIN(maxTiltDeg, wantTilt).

                SET sbDesired TO (COS(sbTiltDeg) * sbUpVec + SIN(sbTiltDeg) * horizCmd):NORMALIZED.
            }
        }
    } ELSE {
        IF sbVHorizMag > 0.1 {
            SET sbTiltDeg TO MIN(maxTiltDeg, sbVHorizMag * 0.5).
            LOCAL sbAnti IS (-1) * sbVHoriz:NORMALIZED.
            SET sbDesired TO (COS(sbTiltDeg) * sbUpVec + SIN(sbTiltDeg) * sbAnti):NORMALIZED.
        }
    }

    LOCAL sbVertFrac IS MAX(0.1, COS(sbTiltDeg)).
    LOCAL sbNetMax IS MAX(0.01, sbMaxAcc * sbVertFrac - sbG).
    LOCAL sbEffAlt IS MAX(0.1, sbAlt - clearance).
    LOCAL sbStopDist IS (sbVVertAbs * sbVVertAbs - touchdownSpeed * touchdownSpeed) /
                        (2 * sbNetMax).
    IF sbStopDist < 0 { SET sbStopDist TO 0. }

    SET sbAccMeas TO 0.
    IF sbDt > 0.01 {
        SET sbAccMeas TO (sbVVert - sbPrevVVert) / sbDt.
    }
    SET sbPrevVVert TO sbVVert.

    SET sbThrottle TO 1.0.
    IF sbEffAlt > sbStopDist * 1.15 + sbVVertAbs * 0.35 {
        LOCAL sbReqDecel IS (sbVVertAbs * sbVVertAbs - touchdownSpeed * touchdownSpeed) /
                            (2 * sbEffAlt).
        IF sbReqDecel < 0 { SET sbReqDecel TO 0. }
        LOCAL sbTargetDecel IS sbReqDecel * 1.10.
        LOCAL sbDesiredAccel IS sbG + sbTargetDecel.
        SET sbThrottle TO clamp(sbDesiredAccel / MAX(0.01, sbMaxAcc * sbVertFrac), 0, 1).
        IF sbAccMeas < sbTargetDecel * 0.65 {
            SET sbThrottle TO 1.0.
        }
    }

    IF sbVVert > 0 AND sbAlt < MAX(50, clearance * 1.5) {
        LOCAL hoverThr IS safeDiv(sbG, MAX(0.01, sbMaxAcc * sbVertFrac), 0.25).
        SET sbThrottle TO MIN(sbThrottle, hoverThr).
    }

    SET sbThrottle TO thrustGate(sbThrottle, sbDesired, maxIgniteTiltDeg).
    SET sbThrottle TO clamp(sbThrottle, 0, 1).
    SET sbSteer TO sbDesired.

    IF sbNow >= sbDiagTimer {
        PRINT "SB alt=" + ROUND(sbAlt) +
              " eff=" + ROUND(sbEffAlt) +
              " stop=" + ROUND(sbStopDist) +
              " vV=" + ROUND(sbVVert,1) +
              " aM=" + ROUND(sbAccMeas,2) +
              " netMax=" + ROUND(sbNetMax,2) +
              " vH=" + ROUND(sbVHorizMag,1) +
              " tilt=" + ROUND(VANG(SHIP:FACING:FOREVECTOR, sbUpVec),1) +
              " thr=" + ROUND(sbThrottle * 100) + "%".
        SET sbDiagTimer TO sbNow + 1.
    }
    WAIT 0.
}

// NOW ASAP should "final landing" comes to work
SET sbThrottle TO 0.
UNLOCK THROTTLE.
UNLOCK STEERING.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.