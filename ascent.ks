// ascent.ks
// Практический ascent для kOS: Q-limited gravity turn + staging + orbit insertion.
PARAMETER targetApoapsis IS 75000.
PARAMETER targetPeriapsis IS -1.
PARAMETER launchHeading IS 90.
PARAMETER maxQ IS 25000.
PARAMETER turnStartAlt IS 250.
PARAMETER turnEndAlt IS 45000.
PARAMETER autoStage IS TRUE.
PARAMETER stageDelay IS 2.
PARAMETER insertionMargin IS 2000.
PARAMETER orbitTolerance IS 500.
PARAMETER fineTuneApoapsis IS TRUE.

IF targetPeriapsis < 0 {
    SET targetPeriapsis TO targetApoapsis.
}
IF targetPeriapsis > targetApoapsis {
    SET targetApoapsis TO targetPeriapsis.
}
IF insertionMargin < 0 {
    SET insertionMargin TO 0.
}

SET ascentAp TO targetApoapsis + insertionMargin.

FUNCTION clamp {
    PARAMETER x.
    PARAMETER lo.
    PARAMETER hi.
    IF x < lo { RETURN lo. }
    IF x > hi { RETURN hi. }
    RETURN x.
}

FUNCTION atmPressure {
    PARAMETER altit.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    LOCAL p IS BODY:ATM:ALTITUDEPRESSURE(altit).
    RETURN MAX(0, p).
}

FUNCTION atmDensity {
    PARAMETER altit.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    // ИСПРАВЛЕНИЕ: 1 atm = 101325 Pa
    LOCAL pPa IS atmPressure(altit) * 101325.
    IF pPa <= 0 { RETURN 0. }
    LOCAL temp IS 288.15 - 0.0065 * altit.
    IF temp < 216.65 { SET temp TO 216.65. }
    RETURN pPa / (287.05 * temp).
}

FUNCTION dynPressure {
    PARAMETER altit.
    PARAMETER veloc.
    IF NOT BODY:ATM:EXISTS { RETURN 0. }
    LOCAL rho IS atmDensity(altit).
    RETURN 0.5 * rho * veloc * veloc.
}

FUNCTION vectorPitch {
    PARAMETER vec.
    IF vec:MAG < 0.1 { RETURN 90. }
    LOCAL sinGamma IS VDOT(vec, SHIP:UP:FOREVECTOR) / vec:MAG.
    SET sinGamma TO clamp(sinGamma, -1, 1).
    RETURN ARCSIN(sinGamma).
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
    IF hasSolid AND sf <= 0.1 { RETURN FALSE. }
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

CLEARSCREEN.
SET SAS TO FALSE.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
SET throttleSet TO 0.
LOCK THROTTLE TO throttleSet.
SET steerPitch TO 90.
SET steerHeading TO launchHeading.
SET steerDir TO HEADING(steerHeading, steerPitch).
LOCK STEERING TO steerDir.

SET phase TO "ascent".
SET lastTime TO TIME:SECONDS.
SET nextStageTime TO TIME:SECONDS + 1.
SET nextPrint TO 0.
SET launchAlt TO SHIP:ALTITUDE.
SET relAlt TO 0.
SET horizSpeed TO 0.
SET vertSpeed TO 0.
SET vOrbNow TO 0.
SET targetPitch TO 90.

IF SHIP:STATUS = "PRELAUNCH" {
    STAGE.
    SET nextStageTime TO TIME:SECONDS + stageDelay.
    SET t0 TO TIME:SECONDS.
    WAIT UNTIL SHIP:MAXTHRUST > 0 OR TIME:SECONDS > t0 + 5.
}

UNTIL phase = "done" {
    SET now TO TIME:SECONDS.
    SET dt TO now - lastTime.
    IF dt <= 0 { SET dt TO 0.05. }
    IF dt > 0.5 { SET dt TO 0.5. }
    SET lastTime TO now.
    
    SET altit TO SHIP:ALTITUDE.
    SET relAlt TO altit - launchAlt.
    SET ap TO SHIP:ORBIT:APOAPSIS.
    SET pe TO SHIP:ORBIT:PERIAPSIS.
    SET speed TO SHIP:AIRSPEED.
    SET pressure TO atmPressure(altit).
    SET dynPres TO dynPressure(altit, SHIP:AIRSPEED).
    SET vOrbNow TO SQRT(BODY:MU / (BODY:RADIUS + altit)).
    SET vertSpeed TO SHIP:VERTICALSPEED.
    SET horizSpeed TO SQRT(MAX(0, speed * speed - vertSpeed * vertSpeed)).
    
    IF autoStage AND now >= nextStageTime AND shouldStage() {
        SET throttleSet TO 0.
        STAGE.
        SET nextStageTime TO now + stageDelay.
        WAIT 0.5.
        IF phase = "ascent" { SET throttleSet TO 1. }
    }
    
    IF phase = "ascent" {
        SET readyForCoast TO (relAlt > turnEndAlt * 0.6) OR (horizSpeed > 0.8 * vOrbNow) OR (ABS(vertSpeed) < 30).
        
        IF pe >= targetPeriapsis - 100 AND ap <= targetApoapsis + orbitTolerance {
            SET throttleSet TO 0.
            SET phase TO "done".
        } ELSE IF ap >= ascentAp AND readyForCoast {
            SET throttleSet TO 0.
            SET phase TO "coastToAp".
        } ELSE {
            SET throttleSet TO 1.
            IF dynPres > maxQ {
                SET throttleSet TO clamp(maxQ / dynPres, 0.05, 1).
            }
            
            IF relAlt < turnStartAlt {
                SET targetPitch TO 90.
            } ELSE {
                SET altFrac TO clamp((relAlt - turnStartAlt) / MAX(1, turnEndAlt - turnStartAlt), 0, 1).
                SET vFrac TO clamp(horizSpeed / vOrbNow, 0, 1).
                SET pitchAlt TO 90 - 90 * altFrac.
                SET pitchVel TO 90 - 90 * vFrac.
                SET targetPitch TO MIN(pitchAlt, pitchVel).
                IF pressure > 5 { SET targetPitch TO MAX(targetPitch, 8). }
                
                SET proPitch TO vectorPitch(SHIP:SRFPROGRADE:VECTOR).
                SET aoaLimit TO 20.
                IF dynPres > maxQ * 0.5 { SET aoaLimit TO 6. }
                IF dynPres > maxQ { SET aoaLimit TO 4. }
                
                SET targetPitch TO clamp(targetPitch, proPitch - aoaLimit, proPitch + aoaLimit).
                SET targetPitch TO clamp(targetPitch, 2, 90).
            }
            
            SET maxRate TO 4.
            IF dynPres > maxQ * 0.5 { SET maxRate TO 1.5. }
            IF dynPres > maxQ { SET maxRate TO 0.6. }
            
            SET dp TO targetPitch - steerPitch.
            SET dp TO clamp(dp, -maxRate * dt, maxRate * dt).
            SET steerPitch TO steerPitch + dp.
            SET steerDir TO HEADING(steerHeading, steerPitch).
            LOCK STEERING TO steerDir.
        }
    } ELSE IF phase = "coastToAp" {
        SET throttleSet TO 0.
        LOCK STEERING TO PROGRADE.
        
        // Создаем ноду если её нет
        IF ALLNODES:LENGTH = 0 {
            SET eta_ap TO SHIP:ORBIT:ETA:APOAPSIS.
            IF eta_ap < 10 { SET eta_ap TO 10. }
            SET r_circ TO BODY:RADIUS + SHIP:ORBIT:APOAPSIS.
            SET v_circ TO SQRT(BODY:MU / r_circ).
            SET v_ap TO SQRT(BODY:MU * (2/r_circ - 1/SHIP:ORBIT:SEMIMAJORAXIS)).
            SET dV TO MAX(0, v_circ - v_ap).
            
            IF dV > 1 {
                // ИСПРАВЛЕНИЕ: используем TimeSpan для ETA
                SET circNode TO NODE(TimeSpan(eta_ap), 0, 0, dV).
                ADD circNode.
                PRINT "Node created at Ap. Required dV: " + ROUND(dV, 2) + " m/s".
            } ELSE {
                PRINT "Orbit already circular (dV < 1 m/s).".
                SET phase TO "done".
            }
        }
        
        // Расчет времени старта горения
        IF ALLNODES:LENGTH > 0 {
            SET n TO ALLNODES[0].
            SET accel TO MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).
            SET burnTime TO n:DELTAV:MAG / accel.
            SET startBurnETA TO (burnTime / 2) + 5.
            
            // Ускорение времени до старта горения
            IF n:ETA > startBurnETA + 30 {
                SET WARP TO 3.
                WAIT UNTIL n:ETA <= startBurnETA + 30 OR ALLNODES:LENGTH = 0.
                SET WARP TO 0.
            }
            
            // Вызов отдельного скрипта для выполнения ноды
            IF n:ETA <= startBurnETA + 30 {
                PRINT "Calling exec_node.ks to execute maneuver...".
                RUN exec_node.
                SET phase TO "done".
            }
        } ELSE {
            SET phase TO "done".
        }
    }
    
    IF now >= nextPrint {
        CLEARSCREEN.
        PRINT "Phase:      " + phase.
        PRINT "Alt ASL:    " + ROUND(altit) + " m".
        PRINT "Alt AGL:    " + ROUND(relAlt) + " m".
        PRINT "Ap:         " + ROUND(ap) + " m".
        PRINT "Pe:         " + ROUND(pe) + " m".
        PRINT "Speed:      " + ROUND(speed) + " m/s".
        PRINT "Horizontal: " + ROUND(horizSpeed) + " m/s".
        PRINT "Q approx:   " + ROUND(dynPres / 1000) + " kPa".
        PRINT "Pressure:   " + ROUND(pressure * 101.325) + " kPa".
        PRINT "Pitch cmd:  " + ROUND(steerPitch) + " deg".
        PRINT "Throttle:   " + ROUND(throttleSet * 100) + "%".
        SET nextPrint TO now + 1.
    }
    WAIT 0.05.
}

SET throttleSet TO 0.
LOCK STEERING TO PROGRADE.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
UNLOCK THROTTLE.
PRINT "Ascent complete. Ap=" + ROUND(SHIP:ORBIT:APOAPSIS) + " Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS).