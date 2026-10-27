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