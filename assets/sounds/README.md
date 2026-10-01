# Sound effects

Source: [ItsBranK/RocketLeague-Audio](https://github.com/ItsBranK/RocketLeague-Audio)
(Rocket League audio dump, upstream license in LICENSE.upstream — The Unlicense, public domain).

All outputs are PCM s16le WAV at the source's native rate/channels (48 kHz).

| file | source path in that repo | processing |
|---|---|---|
| boost.wav | Boosts/SFX_Boost_Alpha/SFX_Boost_Alpha_0003.ogg | decoded to 48 kHz stereo, 5 ms (240-sample) equal-power loop crossfade |
| motor.wav | Motors/SFX_Motor_MuscleCar02/SFX_Motor_MuscleCar02_0001.ogg | decoded to 48 kHz stereo, 25 ms equal-power loop crossfade, peak-normalized to 0.95 |
| thud.wav | Uncategorized/SFX_Impacts/SFX_Impacts_0722.ogg | decoded to 48 kHz stereo, 70 ms fade-out (from 0.47 s), peak-normalized to 0.95 |
| jump.wav | Uncategorized/SFX_VehicleElements/SFX_VehicleElements_0001.ogg | cropped to first 0.50 s, 100 ms fade-out (from 0.40 s), peak-normalized to 0.95 |
| menu.wav | UserInterface/SFX_UI_MainMenu/SFX_UI_MainMenu_0032.ogg | trimmed to first 0.10 s, 10 ms fade-out (from 0.09 s) |

`thud.wav` plays for both ball-vs-arena impacts and car-vs-ball hits;
`jump.wav` plays for jumps, double jumps and flips.
