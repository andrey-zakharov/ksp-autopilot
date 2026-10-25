// change_inclination.ks
// Plane change maneuver to match target orbit inclination
// Executes at the nearest ascending or descending node
// Uses exec_node.ks for continuous course correction during burn

CLEARSCREEN.
PRINT "=== Plane Change Maneuver ===".

// Check if target is set
IF NOT TARGET:EXISTS {
    PRINT "ERROR: No target set! Set a target first.".
    EXIT.
}

// Function to compute angular momentum vector from orbital elements
FUNCTION computeH {
    PARAMETER orb.
    SET i TO orb:INCLINATION.
    SET lan TO orb:LONGITUDEOFASCENDINGNODE.
    SET mu TO orb:BODY:MU.
    SET a TO orb:SEMIMAJORAXIS.
    SET e TO orb:ECCENTRICITY.
    
    // Handle edge cases
    IF a <= 0 { RETURN V(0,0,0). }
    IF e >= 1 { RETURN V(0,0,0). }
    
    SET h_mag TO SQRT(mu * a * (1 - e*e)).
    SET hx TO h_mag * SIN(lan) * SIN(i).
    SET hy TO h_mag * (-COS(lan)) * SIN(i).
    SET hz TO h_mag * COS(i).
    RETURN V(hx, hy, hz).
}

// Function to compute true anomaly at a given universal time
FUNCTION trueAnomalyAt {
    PARAMETER orb.
    PARAMETER ut.
    SET M0 TO orb:MEANANOMALYATEPOCH.
    SET epoch TO orb:EPOCH.
    SET period TO orb:PERIOD.
    SET e TO orb:ECCENTRICITY.
    
    IF period <= 0 { RETURN 0. }
    
    // Mean anomaly at time ut (in degrees)
    SET M TO M0 + 360 * (ut - epoch) / period.
    // Normalize to [0, 360)
    SET M TO M - 360 * FLOOR(M / 360).
    
    // Solve Kepler's equation: M = E - e*sin(E) using Newton's method
    SET E TO M.
    FOR iter FROM 1 TO 15 {
        SET dE TO (E - e * SIN(E) - M) / (1 - e * COS(E)).
        SET E TO E - dE.
        IF ABS(dE) < 0.0001 { BREAK. }
    }
    
    // Convert eccentric anomaly to true anomaly
    SET nu TO 2 * ARCTAN2(SQRT(1+e) * SIN(E/2), SQRT(1-e) * COS(E/2)).
    RETURN nu.
}

// Function to compute position vector at a given true anomaly
FUNCTION positionAtNu {
    PARAMETER orb.
    PARAMETER nu.
    SET a TO orb:SEMIMAJORAXIS.
    SET e TO orb:ECCENTRICITY.
    SET i TO orb:INCLINATION.
    SET lan TO orb:LONGITUDEOFASCENDINGNODE.
    SET w TO orb:ARGUMENTOFPERIAPSIS.
    
    SET p TO a * (1 - e*e).
    SET r_mag TO p / (1 + e * COS(nu)).
    
    // Position in orbital plane
    SET x_orb TO r_mag * COS(nu).
    SET y_orb TO r_mag * SIN(nu).
    
    // Rotate to inertial frame: first by argument of periapsis
    SET x1 TO x_orb * COS(w) - y_orb * SIN(w).
    SET y1 TO x_orb * SIN(w) + y_orb * COS(w).
    SET z1 TO 0.
    
    // Then by inclination
    SET x2 TO x1.
    SET y2 TO y1 * COS(i).
    SET z2 TO y1 * SIN(i).
    
    // Then by longitude of ascending node
    SET x3 TO x2 * COS(lan) - y2 * SIN(lan).
    SET y3 TO x2 * SIN(lan) + y2 * COS(lan).
    SET z3 TO z2.
    
    RETURN V(x3, y3, z3).
}

// Compute angular momentum vectors
SET h_ship TO computeH(SHIP:ORBIT).
SET h_tgt TO computeH(TARGET:ORBIT).

IF h_ship:MAG < 0.01 OR h_tgt:MAG < 0.01 {
    PRINT "ERROR: Invalid orbit data.".
    EXIT.
}

SET h_ship_u TO h_ship:NORMALIZED.
SET h_tgt_u TO h_tgt:NORMALIZED.

// Relative inclination
SET relInc TO ARCCOS(VDOT(h_ship_u, h_tgt_u)).

IF relInc < 0.5 {
    PRINT "Orbits already aligned (relInc=" + ROUND(relInc, 2) + "deg). No maneuver needed.".
    EXIT.
}

PRINT "Relative inclination: " + ROUND(relInc, 2) + " deg".

// Line of nodes vector (intersection of orbital planes)
SET n_vec TO VCRS(h_ship_u, h_tgt_u).
IF n_vec:MAG < 0.001 {
    PRINT "ERROR: Orbits are parallel, no node exists.".
    EXIT.
}
SET n_unit TO n_vec:NORMALIZED.

// Find node crossing times by sampling the orbit
SET period TO SHIP:ORBIT:PERIOD.
SET ut_now TO TIME:SECONDS.
SET samples TO 720.  // 0.5 degree steps
SET dt_step TO period / samples.

SET crossings TO LIST().
SET prev_nu TO trueAnomalyAt(SHIP:ORBIT, ut_now).
SET prev_pos TO positionAtNu(SHIP:ORBIT, prev_nu).
SET prev_dot TO VDOT(prev_pos, h_tgt_u).

FOR i FROM 1 TO samples {
    SET ut TO ut_now + i * dt_step.
    SET nu TO trueAnomalyAt(SHIP:ORBIT, ut).
    SET pos TO positionAtNu(SHIP:ORBIT, nu).
    SET curr_dot TO VDOT(pos, h_tgt_u).
    
    // Check for sign change (crossing the target plane)
    IF (prev_dot > 0 AND curr_dot <= 0) OR (prev_dot < 0 AND curr_dot >= 0) {
        // Refine crossing with binary search
        SET lo TO ut - dt_step.
        SET hi TO ut.
        FOR j FROM 1 TO 20 {
            SET mid TO (lo + hi) / 2.
            SET mid_nu TO trueAnomalyAt(SHIP:ORBIT, mid).
            SET mid_pos TO positionAtNu(SHIP:ORBIT, mid_nu).
            SET mid_dot TO VDOT(mid_pos, h_tgt_u).
            IF (prev_dot > 0 AND mid_dot > 0) OR (prev_dot < 0 AND mid_dot < 0) {
                SET lo TO mid.
            } ELSE {
                SET hi TO mid.
            }
        }
        crossings:ADD((lo + hi) / 2).
    }
    SET prev_dot TO curr_dot.
}

IF crossings:LENGTH = 0 {
    PRINT "ERROR: No node crossings found. Orbits may be coplanar or invalid.".
    EXIT.
}

// Pick the closest node (smallest ETA)
SET best_ut TO crossings[0].
FOR i FROM 1 TO crossings:LENGTH - 1 {
    IF crossings[i] < best_ut {
        SET best_ut TO crossings[i].
    }
}
SET eta_node TO best_ut - ut_now.
PRINT "Next node in " + ROUND(eta_node) + " seconds".

// Compute dV at the node
SET nu_node TO trueAnomalyAt(SHIP:ORBIT, best_ut).
SET r_node TO positionAtNu(SHIP:ORBIT, nu_node).
SET r_mag TO r_node:MAG.

// Orbital speed at node (vis-viva equation)
SET mu TO SHIP:ORBIT:BODY:MU.
SET a TO SHIP:ORBIT:SEMIMAJORAXIS.
SET speed_node TO SQRT(mu * (2/r_mag - 1/a)).

// Required dV for plane change: dV = 2 * v * sin(Δi/2)
SET dV_mag TO 2 * speed_node * SIN(relInc / 2).

PRINT "Required dV: " + ROUND(dV_mag, 2) + " m/s".

// Determine burn direction (normal or anti-normal)
// Check velocity direction relative to target plane
// At ascending node (moving "up" relative to target plane), burn anti-normal
// At descending node (moving "down"), burn normal

// Compute velocity at node (perpendicular to position in orbital plane)
SET e TO SHIP:ORBIT:ECCENTRICITY.
SET p TO a * (1 - e*e).
SET v_r TO SQRT(mu/p) * e * SIN(nu_node).  // Radial velocity
SET v_t TO SQRT(mu/p) * (1 + e * COS(nu_node)).  // Tangential velocity

// Velocity vector in orbital plane
SET v_x_orb TO v_r * COS(nu_node) - v_t * SIN(nu_node).
SET v_y_orb TO v_r * SIN(nu_node) + v_t * COS(nu_node).

// Rotate to inertial frame (same as position rotation)
SET i_orb TO SHIP:ORBIT:INCLINATION.
SET lan TO SHIP:ORBIT:LONGITUDEOFASCENDINGNODE.
SET w TO SHIP:ORBIT:ARGUMENTOFPERIAPSIS.

SET v_x1 TO v_x_orb * COS(w) - v_y_orb * SIN(w).
SET v_y1 TO v_x_orb * SIN(w) + v_y_orb * COS(w).

SET v_x2 TO v_x1.
SET v_y2 TO v_y1 * COS(i_orb).
SET v_z2 TO v_y1 * SIN(i_orb).

SET v_x3 TO v_x2 * COS(lan) - v_y2 * SIN(lan).
SET v_y3 TO v_x2 * SIN(lan) + v_y2 * COS(lan).
SET v_z3 TO v_z2.

SET v_node TO V(v_x3, v_y3, v_z3).

// Check if we're at ascending or descending node
SET v_perp TO VDOT(v_node, h_tgt_u).

IF v_perp > 0 {
    // Ascending node: burn anti-normal (negative normal)
    SET normalSign TO -1.
    PRINT "Ascending node: burning anti-normal".
} ELSE {
    // Descending node: burn normal (positive normal)
    SET normalSign TO 1.
    PRINT "Descending node: burning normal".
}

// Create maneuver node
// NODE(time, radial, normal, prograde)
// For pure plane change: radial=0, prograde=0, normal=±dV
SET eta_node_safe TO MAX(10, eta_node).  // Ensure at least 10 seconds

IF ALLNODES:LENGTH > 0 {
    PRINT "Removing existing nodes...".
    FOR n IN ALLNODES {
        REMOVE n.
    }
}

SET incNode TO NODE(TimeSpan(eta_node_safe), 0, normalSign * dV_mag, 0).
ADD incNode.
PRINT "Node created. dV=" + ROUND(dV_mag, 2) + " m/s".
PRINT "Calling exec_node.ks to execute maneuver...".

// Unlock throttle and call exec_node.ks
UNLOCK THROTTLE.
RUN exec_node.

PRINT "Plane change complete!".