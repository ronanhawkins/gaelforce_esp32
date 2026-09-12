// Gaelforce custom SPAD map for spad_map_id=14 (single capture, <=9 zones).
//
// Paste the four defines and two arrays below into tmf8x2x_test_masks.c from
// ams-OSRAM/tmf8820_21_28_tool_SPAD_maps, `make`, `./spad_tool`. The 109 bytes
// it prints for 0x24-0x90 go into kTofSpadMaskBlob. Output kept alongside in
// spad_tool_output.txt. Verified: the tool accepts this with no errors.
//
// Axes, confirmed by the tool's own 3x3 sample reporting 41x32 deg at
// xSize=18/ySize=6:
//   x = 18 SPADs @ 2.41 deg = the 41 deg axis = VERTICAL on the robot
//   y = 10 SPADs @ 5.55 deg = the 52 deg axis = HORIZONTAL on the robot
// Note the arrays are indexed [y][x], so the narrow vertical dimension is the
// INNER one, 8 rows of 2, not 2 rows of 8.
//
// Vertical: 2 SPADs, and offset 0 centres the window (the tool's 2D view puts
// it at x=8,9 of 18). The band is therefore symmetric about the boresight --
// no mechanical tilt, unlike map 7 whose rows all sit 5.1 deg off-axis.
// Full extent 4.81 deg.
//
// Horizontal: 4 zones x 2 SPADs, centred, giving 42.4 deg total. Zone centres
// land at -16.22 / -5.54 / +5.54 / +16.22 deg, NOT the map 7 spacing, so
// kTofZoneOffAxisDeg must move with this.
//
// Channels ascend with y, so zones read left to right. kTofZones assumes a
// zone reports in the slot matching its channel; the tool labels them 2/4/6/8.
//
// Datasheet constraints (DS000693 7.4.1), all checked by the tool:
//   >=2 adjacent SPADs per channel ... 4 each (2x2)                  OK
//   channel 0 reserved                 not used                      OK
//   >=1 channel per TDC, (2|3)(4|5)(6|7)(8|9) ... 2,4,6,8            OK
//   no channel 1 together with 8 or 9 in a row ... ch1 unused        OK
//   mask + offset within 18x12 ... 2x8 centred                       OK

#define TEST_SPAD_MAP_XOFFSET_2 (0)
#define TEST_SPAD_MAP_YOFFSET_2 (0)
#define TEST_SPAD_MAP_XSIZE     (2)
#define TEST_SPAD_MAP_YSIZE     (8)

static const uint8_t testSpadMapChannel [ TEST_SPAD_MAP_XSIZE * TEST_SPAD_MAP_YSIZE ] =
        /* x = 0  1 */
/* y =  7 */ { 8, 8
    /*  6 */ , 8, 8
    /*  5 */ , 6, 6
    /*  4 */ , 6, 6
    /*  3 */ , 4, 4
    /*  2 */ , 4, 4
    /*  1 */ , 2, 2
    /*  0 */ , 2, 2
             };

static const uint8_t testSpadMapEnable [ TEST_SPAD_MAP_XSIZE * TEST_SPAD_MAP_YSIZE ] =
        /* x = 0  1 */
/* y =  7 */ { 1, 1
    /*  6 */ , 1, 1
    /*  5 */ , 1, 1
    /*  4 */ , 1, 1
    /*  3 */ , 1, 1
    /*  2 */ , 1, 1
    /*  1 */ , 1, 1
    /*  0 */ , 1, 1
             };
