/**
 * routing.h — offline-routing prototype (A* on a small synthetic road grid).
 *
 * A "ROUTE" button on the map enters a crosshair pick mode:
 *   tap 1 = start point, tap 2 = stop point, then a confirm prompt:
 *   "Add path?" -> Yes runs A* and draws the path (timing/memory logged),
 *   No saves the selected start/stop to /sdcard/routing_selection.txt.
 *
 * The graph is a synthetic 8-connected grid generated at init (PSRAM) so the
 * A* engine, heap and memory footprint can be validated on the real board
 * before wiring a real OSM road network. ~GRID_W x GRID_H nodes.
 */
#ifndef ROUTING_H_
#define ROUTING_H_

#include <LovyanGFX.hpp>

/* Build the test grid graph in PSRAM. Call once from setup(). */
void routing_init(void);

/* Boot self-test: run A* across the full grid and log visited/time/mem.
 * Validates the engine on-device without needing the touch UI. */
void routing_selftest(void);

/* true while the offline-routing UI is active (PICK_START..DONE). The main
 * loop keeps the map north-up during this window so the screen-fixed path and
 * crosshairs stay aligned (BLE heading-up would otherwise rotate the map out
 * from under them). */
bool routing_active(void);

/* Enable/disable offline (real-road) routing at runtime. ON loads the graph
 * from SD and frees the synthetic grid; OFF unloads it to free PSRAM (faster
 * boot when offline routing isn't needed). Set from the settings panel. */
void routing_set_offline(bool on);

/* Tap handler for the routing state machine. Returns true if the tap was
 * consumed (button / crosshair pick / confirm dialog). */
bool routing_handle_tap(int x, int y);

/* Screen-fixed overlay: ROUTE button, crosshairs, confirm prompt. Called from
 * drawMap() every frame after the HUD (FULL mode only). */
void routing_draw_overlay(LGFX_Sprite &spr);

#endif // ROUTING_H_
