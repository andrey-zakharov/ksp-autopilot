// runpath("0:/boot/copy").
switch to 1. // vessel
LIST FILES IN fileList.
FOR f in fileList {
   DELETEPATH(f).
}
list files.
function fill { parameter name.
	compile "0:/" + name + ".ks" TO "1:/" + name + ".ksm".
}
fill("math").
fill("dis").
fill("stg").
fill("m_lib").
fill("exec_node").
if SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" OR ship:status = "PRELAUNCH" {
	sas on.
	fill("utils").
	fill("ascent").
} else {
	fill("utils").
	fill("suicide_burn").
	fill("deorbit_phase").
	fill("land").
	fill("horizontal_brake").
	fill("find_landing_site").
}
print "status " + ship:status.
list files.
switch to 0.