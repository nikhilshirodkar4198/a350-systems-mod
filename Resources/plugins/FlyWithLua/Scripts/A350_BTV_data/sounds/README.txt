A350 BTV - sounds
=================

rops/      Voice "REAL" (default): recorded callouts. keep_max_reverse.wav is synthetic, matched to the recording.
voice1/    Voice "Synthetic 1 (US)"     \
voice2/    Voice "Synthetic 2 (UK)"      > generated with Kokoro-82M (Apache-2.0), filtered like a cockpit speaker
voice3/    Voice "Synthetic 3 (US, deep)"/

Each voice has five files:
    runway_too_short.wav   set_max_reverse.wav   keep_max_reverse.wav   brake_max_braking.wav   auto_brake_off.wav
If a file is missing, the same callout is taken from another voice.

Your own sounds
    Create the folder  custom/  here and put files with the names above into it (.wav, .ogg or .mp3).
    Custom files take priority over the voice selected in the settings window.

The sounds play through X-Plane's FMOD on the cockpit (interior) bus: the X-Plane "Interior" volume slider applies,
plus the volume slider in the settings window (FlyWithLua menu -> Macros -> "A350 BTV: Settings").
