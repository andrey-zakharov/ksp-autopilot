LIST FILES IN fileList.
FOR f in fileList {
   DELETEPATH(f).
}
list files.

copypath("0:/utils", "1:/").
copypath("0:/exec_node", "1:/").
if SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" OR ship:status = "PRELAUNCH" {
	sas on.
	copypath("0:/ascent", "1:/").
} else {
	copypath("0:/suicide_burn", "1:/").
	copypath("0:/deorbit_phase", "1:/").
	copypath("0:/land", "1:/").
	copypath("0:/horizontal_brake", "1:/").
	copypath("0:/find_landing_site", "1:/").
}
print "status " + ship:status.
list files.