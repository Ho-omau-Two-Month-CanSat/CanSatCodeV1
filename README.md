# CanSatCodeV1
CanSat Code V1

Code for testing components

UPDATES:

10/14/23 (V9)

- Updated pin assignments to reflect correct pins as connected on PCB
- Added additional MOSFET for LED
- Fixed servo code to activate the feedback servo when separate is true
- Fixed parachute release servo to rotate 180 degrees at 250m AGL
- Added code to manually attach servos if accidental activation occurs
- First flight ready software

10/2/23 (V8)

  - Fixed groundHeightSet
  - Fixed heightAGL and height using BMP390 oversampling and built in function
  - Fixed updating states based on height
  - Fixed telemetry mission time format (changed from displaying milliseconds to hundreths of a second as described in mission guide)
  - Added four ground station commands: (r - release, z - reset ground level, l - launch ready, p - parachute manual release)
