// Gaelforce custom SPAD map for spad_map_id=14 (single capture, <=9 zones).
//
// Input for ams-OSRAM/tmf8820_21_28_tool_SPAD_maps. Paste into
// tmf8x2x_test_masks.c, `make`, `./spad_tool`, and take the I2C sequence.
// We do NOT hand-encode SPAD_ENABLE/SPAD_TDC -- the packing is specified in
// TMF882X_Host_Driver_Communication.pdf, which we do not have. Let the tool do it.
//
// Geometry (DS000693 Fig 29: focal 400um, SPAD x=16.8um, y=38.8um):
//   mask-x = 18 SPADs @ 2.41 deg = the 41 deg axis = VERTICAL on the robot
//   mask-y = 10 SPADs @ 5.55 deg = the 52 deg axis = HORIZONTAL on the robot
//
// Vertical: rows 9+10 only. 18 is even, so the optical axis falls between them
// and the band is symmetric about the boresight -- no mechanical tilt, unlike
// map 7 whose 4 rows all sit off-axis. Full extent 4.81 deg.
//
// Horizontal: 4 zones x 2 SPADs = 8 of 10 columns, centred (cols 2..9).
// Zone centres land at -16.22 / -5.54 / +5.54 / +16.22 deg.
// THIS IS NOT the map-7 spacing -- kTofZoneOffAxisDeg must change to match.
//
// Datasheet constraint checklist (DS000693 7.4.1):
//   >=2 adjacent SPADs per channel ... 4 each (2x2), adjacent        OK
//   channel 0 reserved                 not used                      OK
//   >=1 channel per TDC, (2|3)(4|5)(6|7)(8|9) ... 2,4,6,8            OK
//   no channel 1 together with 8 or 9 in a row ... ch1 unused        OK
//   mask + offset within 18x12 ... 2x8 centred                       OK
//   enable mask and TDC map same size ... both 2x8                   OK

#define GAELFORCE_X_SIZE   2   // vertical, SPAD rows (mask-x)
#define GAELFORCE_Y_SIZE   8   // horizontal, SPAD cols (mask-y)

// assumes offset 0 centres the mask on the optical axis.
#define GAELFORCE_X_OFFSET_2  0
#define GAELFORCE_Y_OFFSET_2  0

// All 16 SPADs in the 2x8 window are live.
const uint8_t gaelforceEnableSpad[GAELFORCE_X_SIZE][GAELFORCE_Y_SIZE] = {
    { 1, 1, 1, 1, 1, 1, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1 },
};

// Four zones left-to-right on channels 2,4,6,8.
const uint8_t gaelforceTdcChannel[GAELFORCE_X_SIZE][GAELFORCE_Y_SIZE] = {
    { 2, 2, 4, 4, 6, 6, 8, 8 },
    { 2, 2, 4, 4, 6, 6, 8, 8 },
};
