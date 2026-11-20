# A350 Systems Mod: OANS · BTV · ROW/ROP · ACARS · CPDLC

**THE MOD WORKS FROM THE SECOND RUN. AFTER THE FIRST RUN, EITHER RESTART X-PLANE OR USE DEVELOPER > RELOAD THE
CURRENT AIRCRAFT AND ART.**


## Installation

1. Put the Resources folder from this download into your X-Plane folder
2. Restart X-Plane.

For ACARS and CPDLC, open Plugins > FlyWithLua > FlyWithLua Macros > A350 Systems once, type your
Hoppie logon code under DATALINK and press Save

You need X-Plane 12 or X-Plane 11 on Windows 10 or 11, the FlightFactor A350 XWB Advanced, and the latest FlyWithLua


## Thank you

Thank you to **Geijapi28** (original A350 BTV) - this mod is heavily built on their work. 


## Biggest changes

- **ZOOM on the range knob** - a new position one step left of 10 shows the airport map on the ND
- **New A/BRK autobrake pushbutton** - next to the gear lever: arms BTV to your chosen exit, or the normal autobrake
  MED, and lights up when armed
- **Airport map (OANS)** - runways, taxiways, stands, markings and OpenStreetMap buildings, styled like the real A350
- **Brake To Vacate (BTV)** - pick an exit on the map, the aircraft brakes to reach it at 10 kt
- **ROW / ROP** - runway overrun warning and protection with stop lines, PFD messages and callouts
- **ATCCOM page like the real A350** - live Hoppie ACARS and CPDLC: logon, requests, clearances with WILCO,
  departure clearance, D-ATIS from VATSIM / IVAO, mailbox, ATC MSG lamps and chime

## About

A systems mod for the FlightFactor A350 XWB Advanced 1.7.4 in X-Plane 12 and X-Plane 11. It adds the airport map
(OANS), Brake To Vacate (BTV), runway overrun warning and protection (ROW/ROP), and live ACARS and CPDLC on the
ATCCOM page. Everything is drawn into the 3D cockpit displays and used with the aircraft's own knobs, pushbuttons
and MFD, as on the real aircraft.

Version 1.0.1 for Windows.

Why the second run: on the first start the mod builds an airport index from your scenery, makes the ND font from
the A350's own display font, and adds the ZOOM position to the range knob and the A/BRK pushbutton to the cockpit
model (backups of the aircraft files are kept). X-Plane has already loaded the cockpit at that moment, so these
appear the next time the A350 is loaded. Nothing from the aircraft is included in this download.


## Updating and uninstalling

If you used an older version: copy the new Resources folder over the old one; your settings are kept. If you had
the A350 Systems Enhancement Mod installed, delete PI_A350_ATCCOM_Backend.py and PI_HoppieBridge.py from
Resources/plugins/PythonPlugins (keep HoppieBridge if another aircraft uses it).
Use only one Hoppie client per callsign, or they take each other's messages.

To uninstall: in the settings window, switch off "ZOOM on the range knob and A/BRK pushbutton" under COCKPIT and
load the A350 once; this restores the aircraft's original cockpit model and textures. Then delete
Resources/plugins/zz_A350_BTV_bridge, and in Resources/plugins/FlyWithLua/Scripts delete A350_BTV.lua, the folders
A350_BTV_data and A350_ATCCOM, and all files whose names start with A350_BTV_. If you forgot to switch the cockpit option off first: in the A350 folder,
copy A350_xp11_cockpit.obj.before_A350_BTV over A350_xp11_cockpit.obj and main_LIT.png.before_A350_BTV over
main_LIT.png.


## Airport map (OANS)

The map appears when the ND is powered, has a valid heading and the IRS are aligned, as on the real aircraft. Turn
the ND range knob to ZOOM, one step left of 10 (mouse wheel or click on the knob). On ZOOM the knob selects the map
range: 5, 2, 1, 0.5 and 0.2 NM; one step past 5 NM goes back to 10. The map is shown in ROSE NAV, ARC and PLAN, on
the ground and in flight; ROSE LS and VOR show the normal ND. ARC is the taxi view (track up, following the
aircraft), ROSE NAV is the same with the full rose, and PLAN is north up and can be moved with the mouse; in flight
PLAN is the BTV view with the landing runway up. The range is the arc radius in ARC and the rose diameter in ROSE
and PLAN, and stays the same when you change the EFIS mode.

While an airport is prepared the ND shows ARPT MAP LOADING for a few seconds. On the first visit the OpenStreetMap
buildings are downloaded in the background as soon as you spawn (avionics and IRS do not matter), usually within a
minute, and appear when they arrive; later visits use the saved copy. If the public servers are busy the download
is tried again after 1, 2 and 4 minutes. The destination and alternate from the F-PLN are downloaded in the
background too, so they are ready when you land. Beyond
0.5 NM the map shows pavement, runways and buildings; markings, labels and shoulders appear from 0.5 NM, and the
taxiway names at every runway turn-off from 1 NM.

The mouse wheel over the map or the range knob changes the range. A double-click opens the menu (ADD CROSS, ADD FLAG,
MAP DATA, CENTER ON ACFT). The double arrow at the bottom right opens the panel with the tabs MAP DATA (find a
runway, taxiway or stand, crosses and flags), LANDING (MODIFY RWY, LDG PERF CODE, runway length, ROW landing
distance, BTV EXIT AT ... M FROM THRESHOLD), ARPT SEL (choose an airport by ICAO, IATA or city) and STATUS. The
airport is chosen automatically: in flight the FMS destination, on the ground the nearest airport. An airport
chosen with DISPLAY AIRPORT stays until the next takeoff or touchdown. One second after touchdown with BTV both NDs
switch to the taxi view by themselves.


## Brake To Vacate (BTV)

To arm BTV in flight: set the A350's autobrake to OFF (BTV cannot be armed while it is on). Open the map, for example
in PLAN. The FMS landing runway is selected automatically; to use another runway, click its name at the runway end.
Click an exit and choose SET BTV EXIT (green exits are recommended, red ones are too short), or enter a stop
distance on the LANDING tab. Then press the A/BRK pushbutton next to the gear lever; it lights up and the FMA shows
BTV in cyan. Without an exit or distance, A/BRK arms the normal autobrake MED instead, and pressing it again disarms
whatever is armed. On the ground the button stays the aircraft's RTO button. Changing the runway, exit, distance,
airport or FMS runway disarms BTV (BTV DISARMED).

After touchdown, when the ground spoilers extend, BTV brakes so that you reach the exit at 10 kt, braking as late as
possible like the real system, and the FMA shows BTV in green. If the nose gear is still in the air, BTV waits for
it (at most 5 seconds). BTV ends at the exit speed; brake further as usual.

You don't need rudder pedals: BTV applies the brakes itself. Pressing the brakes (pedals or X-Plane's brake keys and
buttons), DISARM, pushing the thrust levers forward or retracting the spoilers disconnects BTV, with AUTO BRK OFF
on the PFD, the MASTER CAUT light and chime, and the callout AUTO BRAKE OFF. A go-around below 600 ft disarms BTV
once the aircraft climbs back above 600 ft, as on the A350.


## Runway overrun warning and protection (ROW/ROP)

ROW works on final below 500 ft, also without BTV. With BTV armed, the predicted stop lines are shown on the map
before that, and below 500 ft they update in real time. A WET line beyond the runway end shows IF WET : RWY TOO SHORT
in amber; a DRY line beyond the runway end shows RWY TOO SHORT in red, and below 200 ft you hear RUNWAY TOO SHORT.
The lines use the runway condition from the LANDING tab: with DRY you see a DRY and a WET line, with any other
condition one line for that condition.

ROP is active after touchdown on the selected runway. If even maximum braking would not stop the aircraft before the
runway end, BTV brakes at maximum, the PFD shows MAX REVERSE and you hear SET MAX REVERSE until both reversers are
fully deployed, then KEEP MAX REVERSE below 80 kt. When you brake yourself or use the autobrake, you hear BRAKE MAX
BRAKING until you apply full braking, then the same reverse callouts.


## ACARS and CPDLC (ATCCOM)

Enter your Hoppie logon code once in the settings window (see Installation). Your Hoppie network (VATSIM, IVAO or
NONE for tests) is chosen in your account on hoppie.nl. Power the avionics and enter the flight number in FMS INIT;
it is your datalink callsign. ACARS READY appears within a few seconds. Then choose ATCCOM on the captain's lower
centre MFD (system menu at the top left). The page is used with the mouse, like the real KCCU cursor.

The mailbox is at the top of the right half of the upper centre display, above the memo, as on the A350. It is
always there while the avionics are on, whatever page the MFD shows. Its frame never changes: two side lines, the
message area, a status strip below it and the line above the memo. A message fills the frame: time and station in
green, OPEN or STBY in cyan at the top right, two keys to scroll long text. A message you have prepared but not sent
yet is black on cyan, with SEND and CANCEL on the right; while it is sent it turns green with SENDING and then SENT
in the strip, and a few seconds later the mailbox is empty again. With more than one message the strip shows
MSG 1/2; click it for the next one. PRINT (not available) and RECALL are on the left; RECALL opens the last finished
message again.

The ATCCOM page looks and works like the real A350's: the tabs CONNECT, REQUEST, REPORT & MODIFY, MSG RECORD,
D-ATIS and EMER, a box around the selected tab, a cyan frame around the key under the mouse, amber boxes for
required entries, and messages next to "ATC COM". MSG LIST at the bottom left shows the open messages; while a
message is shown it becomes CLEAR INFO. REPORT & MODIFY, EMER, PRINT and ADS-C are shown but not available. Dropdown lists use FF's list grey. Opening
"ATC COM v" (FF's system menu) shows FF's menu through the page until the mouse leaves the display.

To log on: on CONNECT, type the ATC station in NOTIFY TO ATC or pick one from the list
(the arrow next to the field), and press NOTIFY once. The list shows the 4-character stations logged on to Hoppie
(ATC logon codes; Hoppie also lists every aircraft, which are left out); those staffed on the chosen network right now come first,
marked (VATSIM) or (IVAO), taken from the network's public controller list. The VATSIM / IVAO buttons choose the
network (kept for the next flight); Hoppie itself serves all networks. On VATSIM the controller's info usually gives the logon
code. Wait for LOGON ACCEPTED and ACTIVE ATC; if the station does
not answer, let it time out instead of notifying again. DISCONNECT ALL logs off. Handovers need a manual logoff and a
new logon.

To send a request: on REQUEST, choose a category on the right, then the request, enter the values, press XFR TO
MAILBOX, check it in the mailbox and press SEND. Available are CLIMB TO, DESCEND TO and BLOCK ALT/FL; DIRECT TO,
OFFSET, WX DEVIATION, HEADING and TRACK; SPEED and SPEED RANGE; and WHEN CAN WE EXPECT higher or lower level, climb,
descent, speed or back on route. With a request open, ADD TEXT adds reasons (due to weather, aircraft performance,
turbulence, technical, medical, at pilot's discretion) and your own text. Levels are entered as FL380 or 12000 FT,
speeds as 250 KT or M.78, headings from 001 to 360, offsets as a distance in NM and L or R.

To answer a clearance in the mailbox, press one of its keys and then SEND: ACCEPT (sends WILCO), REJECT (UNABLE) or
STBY for clearances, AFFIRM, NEGATIVE or STBY for questions, ROGER or STBY for information. STBY keeps the message
open. Other messages are closed with CLOSE. Clearances are not loaded into the FMS; fly and check them yourself.
New messages light the ATC MSG pushbuttons and play a chime; pressing ATC MSG acknowledges.

A departure clearance (PDC) is requested on REQUEST > CLEARANCE > DEPARTURE. Departure airport, destination and
aircraft type are filled in; add the gate and ATIS letter if you like, then XFR TO MAILBOX and SEND. It goes to the
departure airport as the usual Hoppie PDC message, without a CPDLC logon, and the clearance arrives in the mailbox.
A real clearance needs a controller online who covers the departure airport (delivery, ground, tower, or an
approach/centre controller above them) and whose software answers Hoppie PDCs (most VATSIM vACCs in Europe with
TopSky). You must be connected to the network with a flight plan filed under the same callsign as FLT NBR, else
the controller's software answers "FLIGHT PLAN NOT HELD - REVERT TO VOICE". The usual answer is first "RCD RECEIVED
... STANDBY", then the clearance (CLD ... CLRD TO ...): press ACCEPT and SEND within a few minutes, or it is
cancelled. Answers from the airport you asked can be answered without a CPDLC logon; any other station's CPDLC
message without a logon is shown read-only with "NO ACTIVE SESSION:" in front.
Free text messages are on REQUEST > OTHER > FREETEXT MSG.

MSG RECORD has ALL MSG and MONITORED MSG. ALL MSG lists everything sent and received, newest first, with its
status; click an entry to see all of it, and RETURN TO LIST to go back. MSG LIST (bottom left) lists the open
messages; click one to open it in the mailbox. D-ATIS has three slots filled from the flight plan (departure, destination, alternate). SEND REQUEST or
UPDATE ALL takes the ATIS from the network chosen on CONNECT (VATSIM or IVAO): on VATSIM from the public list of
online ATIS stations (departure ATIS for the departure slot, arrival ATIS otherwise, or the airport's combined ATIS),
on IVAO from the ATIS text of a controller at that airport. Then it asks Hoppie, and gives the METAR when no ATIS is
online. Only the network's public data is downloaded; nothing about you is sent to VATSIM or IVAO. The datalink needs a flight number
(FLT NBR on the FMS INIT page, or from CPNY F-PLN REQUEST); without one the top line says so. AUTO UPDATE looks for a new ATIS every 10
minutes and only changes the slot when it changed.

Not available: automatic handovers, ADS-C, oceanic clearance, REPORT & MODIFY, EMER, printing, loading clearances
into the FMS, and ATCCOM on the first officer's MFD. Requests use Hoppie's CPDLC wording; how a controller's
software shows the less common ones can differ. CPDLC message numbers are limited to 63 per X-Plane session.


## Settings

Open Plugins > FlyWithLua > FlyWithLua Macros > A350 Systems (or bind the command
FlyWithLua/A350_BTV/toggle_window). It has the airport map on or off, OpenStreetMap buildings on or off, the callout
voice, volume and test, your Hoppie logon code with the connection status, and the cockpit option for the ZOOM
position and A/BRK pushbutton.
Everything else always works like on the real A350: exit speed 10 kt, ROW/ROP and its callouts, MASTER CAUT on BTV
disconnect, and traffic on the map like TCAS.

The callout voice REAL uses recorded A350 callouts (KEEP MAX REVERSE is synthetic, matched to them); three synthetic
voices are also available. The callouts use X-Plane's interior volume. To use your own sounds, put files named
runway_too_short, set_max_reverse, keep_max_reverse, brake_max_braking and auto_brake_off (.wav, .ogg or .mp3) in
Scripts/A350_BTV_data/sounds/custom.

Commands for keys and joystick buttons, all under FlyWithLua/A350_BTV/: toggle_window (settings), arm_toggle (BTV
arm and disarm), disconnect (disconnect BTV; good on the A/THR disconnect button), next_exit and prev_exit,
nd_zoom and nd_zoom_fo (range knob to ZOOM and back), nd_map_in and nd_map_out (map range).


## Troubleshooting

All messages are in X-Plane's Log.txt; search for "A350_BTV", "ND map:" and "Hoppie:". After any change
Your logon code
is never written to the log.

If the map does not appear, check that the ND is powered, the IRS are aligned, the range knob is on ZOOM, the EFIS
mode is ROSE NAV, ARC or PLAN, and the map is switched on in the settings. If the normal ND stays visible on ZOOM,
the helper plugin zz_A350_BTV_bridge is missing or X-Plane was not restarted after installing; the lines "ND map:
draw order" in Log.txt show what happened. If there are no buildings, the airport may have none in OpenStreetMap,
or the download needs internet; a failed download is tried again by itself after 1, 2 and 4 minutes. If there are no callouts, check X-Plane's
interior volume and the volume in the settings, and try Test.

If ACARS is not ready, the DATALINK status in the settings says why: NO LOGON CODE, LOGON CODE REJECTED (typo or a new
code on hoppie.nl), ENTER FLIGHT ID (3 to 7 letters or digits in FMS INIT), CONNECTING (no internet) or READY. If
messages go missing, another Hoppie client is logged on with the same callsign. If a logon is not accepted, the
station must be online and use Hoppie CPDLC. If the ATCCOM page stays FF's own page, the helper plugin is missing
or X-Plane was not restarted.

If FlyWithLua stops all scripts, there was a script error; please send the A350_BTV lines from Log.txt. If X-Plane
crashes the first time the map is drawn, the next start switches off layered ND drawing, and after another crash
the map; both can be switched on again in the settings.


Unofficial community mod. Not affiliated with, endorsed by or supported by Airbus, FlightFactor or Laminar Research.
Airbus, A350, FlightFactor and X-Plane are trademarks of their owners and are used only to describe compatibility.
For flight simulation only.
