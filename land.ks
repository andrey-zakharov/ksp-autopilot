// ==========================================================================
// land.ks — Главный скрипт посадки
// ==========================================================================
parameter landLat is 0.
parameter landLng is 0.
parameter maxSlope is 5.

// Параметры фаз
parameter touchdownSpeed is 0.
parameter finalApproachSpeed is 10.
parameter horizStartAlt is 20000.
parameter brakeLeadTime is 1.1.
parameter groundClearance is 10.
parameter maxIgniteTiltDeg is 15.
parameter suicideTiltDeg is 8.

// Общие параметры
parameter maxSlewDeg is 10.
parameter coastLeadTime is 20.
parameter maxWarpAim is 6.
parameter maxWarpRate is 2.
// Минимальная горизонтальная скорость, которую имеет смысл гасить.
// Ниже этого порога не тратим топливо — просто выравниваемся вертикально.
parameter minHorizBrake is 3.0.
// Максимальный наклон от вертикали у самой точки торможения (градусы).
parameter maxTiltAtBrake is 3.

run utils.

// ==========================================================================
// ГЛОБАЛЬНОЕ СОСТОЯНИЕ ДЛЯ ОБМЕНА МЕЖДУ СКРИПТАМИ
// ==========================================================================
set throttleCmd to 0.
set steerFacing to SHIP:FACING.
set currentWarp to 0.
set lastGlobalSec to TIME:SECONDS.

// Переменные состояния полёта
set altAGL to 0.
set velVert to 0.
set velHorizMag to 0.
set velHorizVec to V(0, 0, 0).
set gravAcc to 0.001.
set thrustAcc to 0.01.
set twrNow to 1.
set brakeAlt to -1.
set timeToBrake to 9999.
set faceRate to 0.

// Координаты площадки
set siteLat to landLat.
set siteLng to landLng.
set siteTerrain to 0.
set nextDiagTimer to 0.

set phase to "".

local doDeorbitCorrection is false.

// ==========================================================================
// ФУНКЦИИ МОНИТОРИНГА
// ==========================================================================
function flightState {
    local nowSec to TIME:SECONDS.
    local rawDt to nowSec - lastGlobalSec.

    set altAGL to MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).

    local upVector to SHIP:UP:FOREVECTOR.
    local velSurf to SHIP:VELOCITY:SURFACE.
    set velVert to VDOT(velSurf, upVector).

    local velHoriz to velSurf - (velVert * upVector).
    set velHorizMag to velHoriz:MAG.
    set velHorizVec to velHoriz.

    local velVertAbs to ABS(velVert).

    set gravAcc to safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.001).

    local thrustAvail to SHIP:AVAILABLETHRUST.
    if thrustAvail <= 0.01 { set thrustAvail to SHIP:MAXTHRUST. }

    local shipMass to MAX(0.001, SHIP:MASS).
    set thrustAcc to MAX(0.01, thrustAvail / shipMass).
    set twrNow to thrustAcc / MAX(0.001, gravAcc).

    set brakeAlt to -1.
    set timeToBrake to 9999.

    // Дерейтим тягу: реальное торможение может быть хуже идеального расчёта
    local netAcc to MAX(0.05, thrustAcc * 0.95 - gravAcc).

    if velVert < 0 {
        local brakeDist to (velVertAbs * velVertAbs - touchdownSpeed * touchdownSpeed) / (2 * netAcc).
        if brakeDist < 0 { set brakeDist to 0. }

        set brakeAlt to brakeDist +
                        velVertAbs * brakeLeadTime +
                        groundClearance.

        set timeToBrake to (SQRT(velVertAbs * velVertAbs + 2 * gravAcc * MAX(0, altAGL - brakeAlt)) - velVertAbs) / MAX(0.0001, gravAcc).
    }
}

function printDiag {
    print phase + "| AGL:" + ROUND(altAGL) +
          " vV:" + ROUND(velVert, 1) +
          " vH:" + ROUND(velHorizMag, 1) +
          " TWR:" + ROUND(twrNow, 2) +
          " Thr:" + ROUND(throttleCmd * 100) + "%" +
          " brakeAlt:" + ROUND(brakeAlt, 1) + "m" +
          " Warp:" + currentWarp.
}

// ==========================================================================
// ФАЗА ГОРИЗОНТАЛЬНОГО ТОРМОЖЕНИЯ (замена horizontal_brake)
// Предиктивный контроль: минимум топлива, ракета не направляется в землю.
// ==========================================================================
function horizontalBrake {
    flightState().

    // Малая горизонталь — не жжём, просто выравниваемся вертикально.
    // Это даёт время ракете встать носом вверх перед суицидом.
    if velHorizMag <= minHorizBrake {
        set throttleCmd to 0.
        set steerFacing to SHIP:UP:FOREVECTOR.
        return.
    }

    // ------------------------------------------------------------
    // Близость к точке торможения: 0 = далеко, 1 = на месте.
    // Начинаем ограничивать наклон за 3 высоты торможения.
    // ------------------------------------------------------------
    local brakeProx to 0.
    if brakeAlt > 0 and altAGL <= brakeAlt * 2 {
        set brakeProx to clamp(1 - (altAGL - brakeAlt) / (brakeAlt * 2), 0, 1).
    }

    // Максимальный наклон от вертикали:
    //   далеко от торможения  → 85°
    //   близко к торможению   → maxTiltAtBrake (≈12°)
    local maxTilt to 85 * (1 - brakeProx) + maxTiltAtBrake * brakeProx.

    // Направление торможения: противоположно горизонтальной скорости
    local brakeDir to (-1) * velHorizVec:NORMALIZED.

    // Ограничиваем наклон от вертикали
    local upDir to SHIP:UP:FOREVECTOR.
    if VANG(brakeDir, upDir) > maxTilt {
        local axV to VCRS(upDir, brakeDir).
        if vecIsOk(axV) and axV:MAG > 0.000001 {
            set brakeDir to (ANGLEAXIS(maxTilt, axV:NORMALIZED) * upDir):NORMALIZED.
        } else {
            set brakeDir to upDir.
        }
    }
    //print maxTilt + " " + ROUND(VANG(brakeDir, upDir), 2).

    // Требуемое ускорение
    local tGround to 9999.
    if velVert < -0.1 {
        set tGround to altAGL / MAX(0.1, ABS(velVert)).
    }
    local availBrake to MAX(0.1, thrustAcc * 0.85).
    local tStop to velHorizMag / availBrake.
    local urgency to 0.
    if tGround < 9999 and tStop > 0.1 {
        set urgency to tStop / MAX(0.1, tGround).
    }

    local wantAcc to 0.
    if urgency > 0.8 {
        set wantAcc to velHorizMag / MAX(0.5, tGround * 0.6).
    } else {
        set wantAcc to velHorizMag / MAX(2, tStop * 1.5).
    }
    set wantAcc to clamp(wantAcc, 0, thrustAcc * 0.9).

    // Тяга: при малых ускорениях не жжём вообще
    if wantAcc < 0.1 {
        set throttleCmd to 0.
    } else {
        set throttleCmd to clamp(wantAcc / MAX(0.1, thrustAcc), 0, 1).
    }

    set steerFacing to safeLookDirUp(brakeDir, SHIP:UP:FOREVECTOR).
}

// ==========================================================================
// СТАРТ
// ==========================================================================
clearscreen.
sas off.
set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
set throttleCmd to 0.
set steerFacing to SHIP:FACING.

lock THROTTLE to throttleCmd.
lock STEERING to steerFacing.

flightState().

print "=== ПОСАДКА ===".
print "Статус: " + SHIP:STATUS + " AGL=" + ROUND(altAGL).

// 1. Площадка: если есть цель — она уже размещена игроком, поиск не нужен
if HASTARGET {
    set siteLat to TARGET:GEOPOSITION:LAT.
    set siteLng to TARGET:GEOPOSITION:LNG.
    set siteTerrain to BODY:GEOPOSITIONLATLNG(siteLat, siteLng):TERRAINHEIGHT.
    print "Цель: " + TARGET:NAME + " " + ROUND(siteLat, 5) + "," + ROUND(siteLng, 5).
    set doDeorbitCorrection to true.
} else {
    run find_landing_site(landLat, landLng, maxSlope).
    set siteLat to landingLat.
    set siteLng to landingLng.
    set siteTerrain to landingTerrain.
    print "Площадка: " + ROUND(siteLat, 5) + "," + ROUND(siteLng, 5).
}

print "Рельеф цели: " + ROUND(siteTerrain) + " м".
wait 1.

lock isLanded to SHIP:STATUS = "LANDED" or SHIP:STATUS = "SPLASHED".
lock fallingDeep to (velVert < 0 and altAGL <= brakeAlt).

// 2. Деорбит (если нужны)
if SHIP:ORBIT:PERIAPSIS > 0 and SHIP:VELOCITY:ORBIT:MAG > 5 {
    print "Деорбит: pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) +
          " v=" + ROUND(SHIP:VELOCITY:ORBIT:MAG, 1).
    run deorbit_phase(horizStartAlt, 100, 48, 8, 1, 1.0, 3000, doDeorbitCorrection).

    sas off.
    set steerFacing to SHIP:FACING.
    lock THROTTLE to throttleCmd.
    lock STEERING to steerFacing.
    wait 1.
}

// ==========================================================================
// 3. ОСНОВНОЙ ЦИКЛ ПОСАДКИ
// ==========================================================================
until isLanded {
    flightState().
    set lastGlobalSec to TIME:SECONDS.
    set phase to "unknown".

    // --- Фаза HORIZONTAL ---
    until isLanded or
          velHorizMag <= minHorizBrake or
          fallingDeep or
          altAGL <= groundClearance {

        set phase to "horizontal brake".

        flightState().
        horizontalBrake().

        if currentWarp > 0 {
            set WARP to 0.
            set currentWarp to 0.
            wait 0.2.
        }

        wait 0.
    }

    // --- Фаза COAST ---
    until isLanded or
          fallingDeep or
          altAGL <= groundClearance or
          (velHorizMag > 1.0 and altAGL <= horizStartAlt) {
        set phase to "freefall".
        flightState().
        set throttleCmd to 0.

        set steerFacing to SHIP:UP:FOREVECTOR.

        local aimErr to VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:FOREVECTOR).
        local wantWarp to 0.

        if throttleCmd < 0.01 and velVert < -1 and
           aimErr <= maxWarpAim and faceRate <= maxWarpRate and
           timeToBrake > coastLeadTime * 2 {
            if timeToBrake > coastLeadTime * 10 { set wantWarp to 4. }
            else if timeToBrake > coastLeadTime * 5 { set wantWarp to 3. }
            else if timeToBrake > coastLeadTime * 3 { set wantWarp to 2. }
            else { set wantWarp to 1. }
        }

        if wantWarp <> currentWarp {
            set WARP to wantWarp.
            set currentWarp to wantWarp.
            wait 0.2.
        }

        wait 0.
    }

    // --- Фаза SUICIDE ---
    if not isLanded and velVert < 0 and altAGL <= brakeAlt and abs(velVert) > finalApproachSpeed {
        set phase to "suicide burn".
        if currentWarp > 0 {
            set WARP to 0.
            set currentWarp to 0.
            wait 0.2.
        }

        set GEAR to true.
        print "SUICIDE START".

        unlock STEERING.
        unlock THROTTLE.

        run suicide_burn(
            groundClearance,
            finalApproachSpeed,
            suicideTiltDeg,
            touchdownSpeed,
            maxIgniteTiltDeg,
            maxSlewDeg
        ).

        set SAS to false.
        set steerFacing to SHIP:FACING.
        set throttleCmd to 0.
        lock THROTTLE to throttleCmd.
        lock STEERING to steerFacing.
    }

    // --- Фаза FINAL ---
    until isLanded or altAGL > groundClearance {
        set phase to "final".
        flightState().
        set GEAR to true.
        set throttleCmd to 0.

        if twrNow < 1.05 {
            set throttleCmd to 0.
        } else {
            local targetMag to touchdownSpeed + (altAGL / MAX(1, groundClearance)) * (finalApproachSpeed - touchdownSpeed).
            local targetVel to -targetMag.
            local err to targetVel - velVert.
            local accCmd to gravAcc + 1.5 * err.
            if velVert > 0 and altAGL < 30 { set accCmd to MIN(accCmd, gravAcc * 0.35). }
            if accCmd < 0 { set accCmd to 0. }
            set throttleCmd to clamp(accCmd / MAX(0.1, thrustAcc), 0, 1).
        }

        // Направление: вертикаль + небольшое отклонение для гашения горизонтали
        local wantDir to SHIP:UP:FOREVECTOR.
        if velHorizMag > 0.2 {
            // Отклоняемся ПРОТИВ горизонтальной скорости
            local brakeDir to (-1) * velHorizVec:NORMALIZED.
            // Максимальное отклонение: 15 градусов
            local tiltDeg to MIN(15, velHorizMag * 5).
            local upDir to SHIP:UP:FOREVECTOR.
            local axisVec to VCRS(upDir, brakeDir).
            if vecIsOk(axisVec) and axisVec:MAG > 0.000001 {
                set wantDir to (ANGLEAXIS(tiltDeg, axisVec:NORMALIZED) * upDir):NORMALIZED.
            }
        }

        set steerFacing to wantDir.
        wait 0.
    }


    if TIME:SECONDS > nextDiagTimer {
        printDiag().
        set nextDiagTimer to TIME:SECONDS + 2.
    }
}

// ==========================================================================
// ЗАВЕРШЕНИЕ
// ==========================================================================
set throttleCmd to 0.
set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
unlock THROTTLE.
unlock STEERING.
set WARP to 0.
sas on.
set GEAR to true.
print "=== ГОТОВО ===".