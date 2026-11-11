@LAZYGLOBAL ON.

function sumBurnTime { parameter needDeltaV.
    local remainDeltaV to needDeltaV.
    local currentStage to SHIP:STAGENUM.
    local burnTime to 0.
    until remainDeltaV <= 0 or currentStage < 0 {
        local sdv to SHIP:STAGEDELTAV(currentStage).
        //print "stage: " + currentStage + " " + sdv:CURRENT + " " + sdv:DURATION.

        if sdv:CURRENT >= remainDeltaV {
            set burnTime to burnTime + (remainDeltaV * sdv:DURATION / sdv:CURRENT).
            set remainDeltaV to 0.
        } else { // not enough fuel in current stage
            set remainDeltaV to remainDeltaV - sdv:CURRENT.
            set burnTime to burnTime + sdv:DURATION.
        }
        //print burnTime + " " + remainDeltaV.
        
        set currentStage to currentStage - 1.
    }
    return burnTime.
}

function execAim { parameter aimV. parameter timeOutSec is 20.
    local aimH to aimV:NORMALIZED.
    local tEnd to TIME:SECONDS + timeOutSec.

    wait until VANG(SHIP:FACING:FOREVECTOR, aimH) <= 1 or TIME:SECONDS > tEnd.
    PRINT "Aligned (angle=" + ROUND(VANG(SHIP:FACING:FOREVECTOR, aimH), 2) + "deg).".
}